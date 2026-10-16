// SPDX-License-Identifier: LGPL-2.1-or-later

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/MaterialGrid.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialLayout.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialMesher.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialSimulation.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialTool.h>
#include <Mod/CAM/PathSimulator/AppGL/ThreadPool.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator::Material;
using CAMSimulator::ThreadPool;

namespace
{

constexpr float ToolLength = 40;

std::vector<float> FlatProfile(float radius)
{
    return {radius, ToolLength, radius, 0, 0, 0};
}

// a quarter circle in 'segments' edges, from (cr + radius, cz) down to (cr, cz - radius)
void Arc(std::vector<float>& profile, float cr, float cz, float radius, int segments)
{
    for (int i = 1; i <= segments; i++) {
        const float a = i * (float)M_PI / 2 / segments;
        profile.push_back(cr + radius * std::cos(a));
        profile.push_back(cz - radius * std::sin(a));
    }
}

std::vector<float> BallProfile(float radius, int segments = 32)
{
    std::vector<float> profile = {radius, ToolLength, radius, radius};
    Arc(profile, 0, radius, radius, segments);
    return profile;
}

std::vector<float> BullNoseProfile(float radius, float corner)
{
    std::vector<float> profile = {radius, ToolLength, radius, corner};
    Arc(profile, radius - corner, corner, corner, 16);
    profile.push_back(0);
    profile.push_back(0);
    return profile;
}

std::vector<float> VBitProfile(float radius, float halfAngleDeg)
{
    const float h = radius / std::tan(halfAngleDeg * (float)M_PI / 180);
    return {radius, ToolLength, radius, h, 0, 0};
}

// neck radius 3 above height 6, widening to radius 7 at the tip
std::vector<float> DovetailProfile()
{
    return {3, ToolLength, 3, 6, 7, 0, 0, 0};
}

// a disk of radius 8 and height 3 on a neck of radius 2.5, with a stub of radius 2.5 below
std::vector<float> TSlotProfile()
{
    return {2.5f, ToolLength, 2.5f, 3, 8, 3, 8, 0, 2.5f, 0, 2.5f, -1};
}

// a wide head and a wide tip on a thin neck: lines along the axis meet it twice
std::vector<float> WaistedProfile()
{
    return {2, ToolLength, 2, 6, 0.5f, 6, 0.5f, 2, 2, 2, 2, 0, 0, 0};
}

// a probe is modeled below its tip
std::vector<float> ProbeProfile()
{
    return {1, 0, 1, -20, 0, -21};
}

// box stock from lo to hi
void MakeBox(const double lo[3], const double hi[3], std::vector<float>& xyz, std::vector<uint32_t>& triangles)
{
    xyz.clear();
    for (int i = 0; i < 8; i++) {
        xyz.push_back((float)(i & 1 ? hi[0] : lo[0]));
        xyz.push_back((float)(i & 2 ? hi[1] : lo[1]));
        xyz.push_back((float)(i & 4 ? hi[2] : lo[2]));
    }
    triangles = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
}

void Normalize(double v[3])
{
    const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    for (int k = 0; k < 3; k++) {
        v[k] /= l;
    }
}

// Point inside the tool at a time of the motion, the tool grown (grow > 0) or shrunk (grow < 0)
bool InToolAt(const Tool& tool, const Motion& m, double t, const double p[3], double grow)
{
    double tip[3], axis[3], w[3];
    for (int k = 0; k < 3; k++) {
        tip[k] = m.p0[k] + t * (m.p1[k] - m.p0[k]);
        axis[k] = m.a0[k] + t * (m.a1[k] - m.a0[k]);
    }
    if (m.arc) {
        const double a = m.startAngle + t * m.sweep;
        tip[0] = m.center[0] - std::sin(a) * m.radius;
        tip[1] = m.center[1] + std::cos(a) * m.radius;
    }
    Normalize(axis);
    for (int k = 0; k < 3; k++) {
        w[k] = p[k] - tip[k];
    }
    const double z = w[0] * axis[0] + w[1] * axis[1] + w[2] * axis[2];
    const double r = std::sqrt(std::max(0.0, w[0] * w[0] + w[1] * w[1] + w[2] * w[2] - z * z));
    if (grow > 0) {
        // anywhere within 'grow' of the solid
        for (double dz = -grow; dz <= grow; dz += grow / 4) {
            if (r <= tool.RadiusAt(z + dz) + grow) {
                return true;
            }
        }
        return false;
    }
    for (double dz = grow; dz <= -grow; dz -= grow / 4) {
        if (r > tool.RadiusAt(z + dz) + grow) {
            return false;
        }
    }
    return true;
}

// Every point of a line well inside the swept tool is removed, every point well outside is not
void CheckSweep(const Tool& tool, const Motion& m, std::mt19937& rng, int lines)
{
    const Sweep sweep(m);
    std::uniform_real_distribution<double> U(0, 1);
    double length = 0, turn = 0;
    for (int k = 0; k < 3; k++) {
        length += (m.p1[k] - m.p0[k]) * (m.p1[k] - m.p0[k]);
        turn += (m.a1[k] - m.a0[k]) * (m.a1[k] - m.a0[k]);
    }
    length = std::sqrt(length) + std::sqrt(turn) * tool.MaxZ() + std::abs(m.sweep) * m.radius;
    const double tol = 0.01;
    const int times = std::max(2, (int)std::ceil(length / (tol / 4)));
    for (int n = 0; n < lines; n++) {
        const int axis = n % 3;
        double origin[3];
        for (int k = 0; k < 3; k++) {
            origin[k] = sweep.BoxLo()[k] + U(rng) * (sweep.BoxHi()[k] - sweep.BoxLo()[k]);
        }
        const double sMin = sweep.BoxLo()[axis] - 1, sMax = sweep.BoxHi()[axis] + 1;
        std::vector<Removal> out;
        sweep.Line(axis, origin, sMin, sMax, out);
        for (int i = 0; i <= 100; i++) {
            const double s = sMin + (sMax - sMin) * i / 100;
            double p[3] = {origin[0], origin[1], origin[2]};
            p[axis] = s;
            bool removed = false;
            for (const Removal& r : out) {
                removed = removed || (s >= r.lo.s && s <= r.hi.s);
            }
            bool surelyIn = false, maybeIn = false;
            for (int k = 0; k <= times && !surelyIn; k++) {
                const double t = (double)k / times;
                maybeIn = maybeIn || InToolAt(tool, m, t, p, tol);
                surelyIn = surelyIn || InToolAt(tool, m, t, p, -tol);
            }
            if (surelyIn) {
                EXPECT_TRUE(removed) << "point inside the sweep not removed, axis " << axis << " s " << s;
            }
            if (!maybeIn) {
                EXPECT_FALSE(removed) << "point outside the sweep removed, axis " << axis << " s " << s;
            }
        }
    }
}

Motion MakeMotion(const Tool& tool, std::mt19937& rng, int mode, bool turning)
{
    std::uniform_real_distribution<double> U(-1, 1);
    Motion m;
    m.tool = &tool;
    const double R = tool.Radius();
    for (int k = 0; k < 3; k++) {
        m.p0[k] = U(rng) * R;
        m.p1[k] = m.p0[k] + U(rng) * 3 * R;
    }
    double a[3] = {0, 0, 1};
    if (mode == 1) {
        a[0] = 0;
        a[1] = -1;
        a[2] = 0;
    }
    if (mode >= 2) {
        for (double& c : a) {
            c = U(rng);
        }
    }
    Normalize(a);
    // half of them across the axis, the fast paths
    if (rng() % 2) {
        double d[3], along = 0;
        for (int k = 0; k < 3; k++) {
            d[k] = m.p1[k] - m.p0[k];
            along += d[k] * a[k];
        }
        for (int k = 0; k < 3; k++) {
            m.p1[k] -= along * a[k];
        }
    }
    for (int k = 0; k < 3; k++) {
        m.a0[k] = m.a1[k] = a[k];
    }
    if (turning) {
        for (int k = 0; k < 3; k++) {
            m.a1[k] = a[k] + 0.3 * U(rng);
        }
        Normalize(m.a1);
    }
    return m;
}

const std::vector<std::vector<float>>& ToolProfiles()
{
    static const std::vector<std::vector<float>> profiles = {
        FlatProfile(3),
        BallProfile(3),
        BullNoseProfile(4, 1),
        VBitProfile(3, 30),
        DovetailProfile(),
        TSlotProfile(),
        WaistedProfile(),
    };
    return profiles;
}

// The stock of a simulation, at a step, as the inside of random points
std::vector<uint8_t> SampleInside(const Grid& grid, const std::vector<std::array<double, 3>>& points)
{
    const Layout& L = grid.GetLayout();
    std::vector<uint8_t> inside;
    for (const auto& p : points) {
        int units[3];
        for (int k = 0; k < 3; k++) {
            units[k] = (int)std::floor((p[k] - L.Origin()[k]) / L.Unit());
        }
        const int leaf = L.FindLeaf(units[0], units[1], units[2]);
        const Layout::Leaf& lf = L.GetLeaf(leaf);
        // the Z dexel of the leaf nearest to the point
        const int i = std::clamp((int)std::lround((p[0] - L.Coord(0, lf.lo[0])) / (lf.cell * L.Unit())), 0, Cells);
        const int j = std::clamp((int)std::lround((p[1] - L.Coord(1, lf.lo[1])) / (lf.cell * L.Unit())), 0, Cells);
        Span scratch;
        const Span* spans;
        const int n = grid.Line(leaf, LineIndex(2, i, j), spans, scratch);
        uint8_t in = 0;
        for (int s = 0; s < n; s++) {
            // a margin around the ends of the material: 2 = unsure
            if (p[2] > spans[s].lo - 1e-3 && p[2] < spans[s].hi + 1e-3) {
                in = p[2] > spans[s].lo + 1e-3 && p[2] < spans[s].hi - 1e-3 ? 1 : 2;
            }
        }
        inside.push_back(in);
    }
    return inside;
}

Segment Line(int tool, const float a[3], const float b[3], int steps = 10)
{
    Segment s;
    s.tool = tool;
    for (int k = 0; k < 3; k++) {
        s.start[k] = a[k];
        s.end[k] = b[k];
    }
    s.steps = s.sweepSteps = steps;
    return s;
}

// Wait until the simulation shows a step, and copy its stock into 'grid'
void WaitShown(Simulation& sim, long step, Grid& grid)
{
    sim.SetTarget(step);
    std::vector<int> changed;
    for (int i = 0; i < 20000; i++) {
        if (!grid.LayoutRef() && sim.GetLayout()) {
            grid.Init(sim.GetLayout());
        }
        if (grid.LayoutRef() && !sim.Pending() && sim.TakeDisplay(grid, changed) == step) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    FAIL() << "the simulation did not show step " << step;
}

}  // namespace

TEST(MaterialTool, primitives)
{
    const Tool ball(BallProfile(3));
    ASSERT_EQ(ball.Spheres().size(), 1u);
    EXPECT_NEAR(ball.Spheres()[0].zc, 3, 1e-5);
    EXPECT_NEAR(ball.Spheres()[0].radius, 3, 1e-5);
    EXPECT_TRUE(ball.SingleAxial());
    EXPECT_TRUE(ball.Cuts());

    // the corner of a bull nose is not centered on the axis: frustums
    const Tool bull(BullNoseProfile(4, 1));
    EXPECT_TRUE(bull.Spheres().empty());
    EXPECT_GT(bull.Frustums().size(), 10u);

    EXPECT_TRUE(Tool(TSlotProfile()).SingleAxial());
    EXPECT_FALSE(Tool(WaistedProfile()).SingleAxial());
    EXPECT_TRUE(Tool(DovetailProfile()).SingleAxial());
    EXPECT_FALSE(Tool(ProbeProfile()).Cuts());

    // the lowest point at a distance from the axis
    EXPECT_NEAR(ball.BottomAt(0), 0, 1e-6);
    EXPECT_NEAR(ball.BottomAt(2), 3 - std::sqrt(5.0), 1e-4);
    EXPECT_NEAR(Tool(VBitProfile(3, 45)).BottomAt(1), 1, 1e-6);
}

TEST(MaterialTool, sweepsOfFixedAxes)
{
    std::mt19937 rng(1);
    for (const auto& profile : ToolProfiles()) {
        const Tool tool(profile);
        for (int i = 0; i < 12; i++) {
            CheckSweep(tool, MakeMotion(tool, rng, i % 4, false), rng, 3);
        }
    }
}

TEST(MaterialTool, sweepsOfTurningAxes)
{
    std::mt19937 rng(2);
    for (const auto& profile : ToolProfiles()) {
        const Tool tool(profile);
        for (int i = 0; i < 4; i++) {
            SCOPED_TRACE(testing::Message() << "tool radius " << tool.Radius() << " motion " << i);
            CheckSweep(tool, MakeMotion(tool, rng, i % 4, true), rng, 3);
        }
    }
}

TEST(MaterialTool, sweepsOfArcs)
{
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> U(-1, 1);
    for (const auto& profile : ToolProfiles()) {
        const Tool tool(profile);
        for (int i = 0; i < 8; i++) {
            Motion m;
            m.tool = &tool;
            m.arc = true;
            m.center[0] = U(rng) * tool.Radius();
            m.center[1] = U(rng) * tool.Radius();
            m.radius = (0.2 + U(rng) * 0.1 + (i % 3)) * tool.Radius();
            m.startAngle = U(rng) * M_PI;
            m.sweep = U(rng) * M_PI / 2;
            m.p0[2] = m.p1[2] = U(rng);
            for (int e = 0; e < 2; e++) {
                const double a = m.startAngle + e * m.sweep;
                double* p = e ? m.p1 : m.p0;
                p[0] = m.center[0] - std::sin(a) * m.radius;
                p[1] = m.center[1] + std::cos(a) * m.radius;
            }
            if (i % 2) {
                // pointing down, like a tool of a work plane from below
                m.a0[2] = m.a1[2] = -1;
            }
            CheckSweep(tool, m, rng, 6);
        }
    }
}

TEST(MaterialTool, normalsPointOutOfTheMaterial)
{
    // a flat end mill moving along X at a depth: floor and walls
    const Tool tool(FlatProfile(2));
    Motion m;
    m.tool = &tool;
    m.p0[0] = 0;
    m.p1[0] = 10;
    m.p0[2] = m.p1[2] = -3;
    const Sweep sweep(m);
    std::vector<Removal> out;
    const double z[3] = {5, 0.5, 0};
    ASSERT_EQ(sweep.Line(2, z, -10, 10, out), 1);
    EXPECT_NEAR(out[0].lo.s, -3, 1e-9);
    float n[3];
    ASSERT_TRUE(sweep.Normal(2, z, out[0].lo, n));
    EXPECT_NEAR(n[2], 1, 1e-6);  // the floor faces up
    out.clear();
    const double y[3] = {5, 0, -1};
    ASSERT_EQ(sweep.Line(1, y, -10, 10, out), 1);
    EXPECT_NEAR(out[0].lo.s, -2, 1e-9);
    EXPECT_NEAR(out[0].hi.s, 2, 1e-9);
    ASSERT_TRUE(sweep.Normal(1, y, out[0].lo, n));
    EXPECT_NEAR(n[1], 1, 1e-6);  // the wall at y = -2 faces +y
    ASSERT_TRUE(sweep.Normal(1, y, out[0].hi, n));
    EXPECT_NEAR(n[1], -1, 1e-6);
}

TEST(MaterialGrid, slotIsExact)
{
    ThreadPool pool(4);
    const double lo[3] = {0, 0, -10}, hi[3] = {20, 20, 0};
    Layout::Builder builder(lo, hi, 0.5, 1, pool.Size());
    const double a[3] = {0, 10, -3}, b[3] = {20, 10, -3};
    builder.AddCapsule(0, a, b, 4, 1);
    const std::shared_ptr<Layout> layout = builder.Build(pool);
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    Grid grid;
    grid.Init(layout);
    grid.SetMesh(xyz, triangles, pool);

    const Tool tool(FlatProfile(2));
    Motion m;
    m.tool = &tool;
    m.p0[0] = -5;
    m.p1[0] = 25;
    m.p0[1] = m.p1[1] = 10;
    m.p0[2] = m.p1[2] = -3;
    m.tag = 5;
    const Sweep sweep(m);
    grid.Cut({&sweep}, pool);

    // every dexel: the material is outside the slot, the cut ends have the normals of the slot
    int cutEnds = 0;
    for (int leaf = 0; leaf < layout->LeafCount(); leaf++) {
        for (int line = 0; line < BrickLines; line++) {
            Span scratch;
            const Span* spans;
            const int n = grid.Line(leaf, line, spans, scratch);
            double o[3];
            grid.LineOrigin(leaf, line, o);
            const int axis = line / AxisLines;
            for (int s = 0; s < n; s++) {
                for (int e = 0; e < 2; e++) {
                    const uint32_t packed = e ? spans[s].nhi : spans[s].nlo;
                    if (!IsCut(packed)) {
                        continue;
                    }
                    cutEnds++;
                    EXPECT_EQ(TagOf(packed), 5);
                    double p[3] = {o[0], o[1], o[2]};
                    p[axis] = e ? spans[s].hi : spans[s].lo;
                    // on the floor or a wall of the slot
                    const bool floor = std::abs(p[2] + 3) < 1e-4 && std::abs(p[1] - 10) <= 2 + 1e-4;
                    const bool wall = std::abs(std::abs(p[1] - 10) - 2) < 1e-4 && p[2] >= -3 - 1e-4;
                    EXPECT_TRUE(floor || wall) << p[0] << " " << p[1] << " " << p[2];
                    if (HasNormal(packed)) {
                        float n3[3];
                        UnpackNormal(packed, n3);
                        if (floor && !wall) {
                            EXPECT_NEAR(n3[2], 1, 1e-3);
                        }
                        if (wall && !floor) {
                            EXPECT_NEAR(std::abs(n3[1]), 1, 1e-3);
                        }
                    }
                }
            }
        }
    }
    EXPECT_GT(cutEnds, 100);
}

TEST(MaterialSimulation, seeksMatchTheWholePath)
{
    // a pocket with arcs and ramps, cut by two tools
    std::vector<std::shared_ptr<const Tool>> tools = {
        std::make_shared<Tool>(FlatProfile(2)),
        std::make_shared<Tool>(BallProfile(1.5f)),
    };
    std::vector<Segment> segments;
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> U(2, 18);
    float at[3] = {10, 10, 1};
    for (int i = 0; i < 120; i++) {
        float next[3] = {U(rng), U(rng), -(float)(i % 5) - 0.5f};
        segments.push_back(Line(i < 60 ? 0 : 1, at, next, 3 + i % 7));
        std::copy(next, next + 3, at);
        if (i % 10 == 0) {
            // an arc around the current point
            Segment arc;
            arc.tool = i < 60 ? 0 : 1;
            arc.arc = true;
            arc.center[0] = at[0] - 2;
            arc.center[1] = at[1];
            arc.radius = 2;
            arc.startAngle = -(float)M_PI / 2;
            arc.sweep = (float)M_PI;
            arc.start[0] = at[0];
            arc.start[1] = at[1];
            arc.start[2] = arc.end[2] = at[2];
            arc.end[0] = at[0] - 4;
            arc.end[1] = at[1];
            arc.steps = arc.sweepSteps = 8;
            segments.push_back(arc);
            at[0] -= 4;
        }
    }
    const double lo[3] = {0, 0, -6}, hi[3] = {20, 20, 0};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);

    std::vector<std::array<double, 3>> points;
    std::uniform_real_distribution<double> P(0, 1);
    for (int i = 0; i < 4000; i++) {
        points.push_back({lo[0] + P(rng) * 20, lo[1] + P(rng) * 20, lo[2] + P(rng) * 6});
    }
    Simulation sim;
    sim.Start(tools, segments, xyz, triangles, 5);
    const long total = sim.TotalSteps();
    // to the end first, so the targets below come from checkpoints, forwards and backwards
    Grid grid;
    WaitShown(sim, total, grid);
    sim.WaitFrontier();
    const std::vector<long> targets = {total / 2, total / 3, total - 5, 7, total / 2 + 3, total / 2 + 1};
    for (long target : targets) {
        WaitShown(sim, target, grid);
        // the same step simulated from the start, in a new simulation
        Simulation direct;
        direct.Start(tools, segments, xyz, triangles, 5);
        Grid directGrid;
        WaitShown(direct, target, directGrid);
        const std::vector<uint8_t> a = SampleInside(grid, points), b = SampleInside(directGrid, points);
        int differ = 0;
        for (size_t i = 0; i < a.size(); i++) {
            differ += a[i] != 2 && b[i] != 2 && a[i] != b[i];
        }
        EXPECT_EQ(differ, 0) << "at step " << target;
    }
}

TEST(MaterialMesher, closedAndOriented)
{
    ThreadPool pool(4);
    std::vector<std::shared_ptr<const Tool>> tools = {
        std::make_shared<Tool>(FlatProfile(2)),
        std::make_shared<Tool>(BallProfile(1)),
    };
    const float a[3] = {4, 10, -2}, b[3] = {16, 10, -2}, c[3] = {16, 4, -3.5f}, d[3] = {4, 16, -1};
    std::vector<Segment> segments = {Line(0, a, b), Line(1, b, c), Line(1, c, d)};
    const double lo[3] = {0, 0, -6}, hi[3] = {20, 20, 0};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    Simulation sim;
    sim.Start(tools, segments, xyz, triangles, 6);
    Grid grid;
    WaitShown(sim, sim.TotalSteps(), grid);
    Mesher mesher;
    mesher.Reset(sim.GetLayout(), pool);
    std::vector<int> all;
    mesher.Update(grid, all, pool);

    // vertices by position, every directed edge has its reverse: closed and oriented
    std::map<std::array<float, 3>, int> ids;
    std::map<std::pair<int, int>, int> edges;
    double volume = 0;
    for (int u = 0; u < mesher.UnitCount(); u++) {
        const UnitMesh& mesh = mesher.Mesh(u);
        for (const auto* list : {&mesh.stock, &mesh.cut}) {
            for (size_t t = 0; t + 2 < list->size(); t += 3) {
                int id[3];
                const MeshVertex* v[3];
                for (int k = 0; k < 3; k++) {
                    v[k] = &mesh.vertices[(*list)[t + k]];
                    const std::array<float, 3> key = {v[k]->x, v[k]->y, v[k]->z};
                    id[k] = ids.emplace(key, (int)ids.size()).first->second;
                }
                for (int k = 0; k < 3; k++) {
                    edges[{id[k], id[(k + 1) % 3]}]++;
                }
                volume += (v[0]->x * (v[1]->y * v[2]->z - v[1]->z * v[2]->y)
                           - v[0]->y * (v[1]->x * v[2]->z - v[1]->z * v[2]->x)
                           + v[0]->z * (v[1]->x * v[2]->y - v[1]->y * v[2]->x))
                    / 6;
            }
        }
    }
    int open = 0;
    for (const auto& [e, n] : edges) {
        auto it = edges.find({e.second, e.first});
        open += it == edges.end() || it->second != n;
    }
    EXPECT_EQ(open, 0);
    // less than the box, by more than the slot of the flat end mill alone
    EXPECT_LT(volume, 20.0 * 20 * 6 - 12 * 4 * 2 + 1);
    EXPECT_GT(volume, 20.0 * 20 * 6 - 400);
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
