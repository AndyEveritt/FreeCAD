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
#include <Mod/CAM/PathSimulator/AppGL/MaterialModel.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialSimulation.h>
#include <Mod/CAM/PathSimulator/AppGL/MaterialTool.h>
#include <Mod/CAM/PathSimulator/AppGL/ThreadPool.h>

#include "MaterialTestTools.h"

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator::Material;
using CAMSimulator::ThreadPool;
using namespace MaterialTest;

namespace
{

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
                EXPECT_TRUE(removed)
                    << "point inside the sweep not removed, axis " << axis << " s " << s;
            }
            if (!maybeIn) {
                EXPECT_FALSE(removed)
                    << "point outside the sweep removed, axis " << axis << " s " << s;
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
        const int i = std::clamp(
            (int)std::lround((p[0] - L.Coord(0, lf.lo[0])) / (lf.cell * L.Unit())),
            0,
            Cells
        );
        const int j = std::clamp(
            (int)std::lround((p[1] - L.Coord(1, lf.lo[1])) / (lf.cell * L.Unit())),
            0,
            Cells
        );
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

// Edges of the meshes of all units without their reverse: 0 for a closed, oriented mesh. The
// signed volume enclosed, if asked for.
int OpenEdges(const Mesher& mesher, double* volume = nullptr)
{
    std::map<std::array<float, 3>, int> ids;
    std::map<std::pair<int, int>, int> edges;
    double enclosed = 0;
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
                enclosed += (v[0]->x * (v[1]->y * v[2]->z - v[1]->z * v[2]->y)
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
    if (volume) {
        *volume = enclosed;
    }
    return open;
}

// Vertices of the meshes of all units on their cut faces
template<typename F>
void ForCutVertices(const Mesher& mesher, F f)
{
    for (int u = 0; u < mesher.UnitCount(); u++) {
        const UnitMesh& mesh = mesher.Mesh(u);
        for (uint32_t i : mesh.cut) {
            f(mesh.vertices[i]);
        }
    }
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

TEST(MaterialTool, arcsOfTheProfileAreExact)
{
    // a bull nose corner given in 4 lines, and as an arc around (3, 2): the tool follows the circle
    std::vector<float> profile = {5, ToolLength, 5, 2};
    Arc(profile, 3, 2, 2, 4);
    profile.insert(profile.end(), {0, 0});
    const Tool coarse(profile);
    const Tool exact(profile, true, {{1, 5, 3, 2}});
    auto error = [](const Tool& tool) {
        double worst = 0;
        for (int i = 1; i < 200; i++) {
            const double z = 2.0 * i / 200;
            worst = std::max(worst, std::abs(std::hypot(tool.RadiusAt(z) - 3, z - 2) - 2));
        }
        return worst;
    };
    EXPECT_GT(error(coarse), 0.01);
    EXPECT_LT(error(exact), Tool::ArcTolerance + 1e-6);
    EXPECT_NEAR(exact.RadiusAt(2.5), 5, 1e-6);
    EXPECT_NEAR(exact.BottomAt(1), 0, 1e-6);

    // a ball end in 2 lines is no sphere, as an arc it is
    const std::vector<float> ball = BallProfile(3, 2);
    EXPECT_TRUE(Tool(ball).Spheres().empty());
    const Tool sphere(ball, true, {{1, 3, 0, 3}});
    ASSERT_EQ(sphere.Spheres().size(), 1u);
    EXPECT_NEAR(sphere.Spheres()[0].zc, 3, 1e-6);
    EXPECT_NEAR(sphere.Spheres()[0].radius, 3, 1e-6);

    // arcs out of the profile are left out
    EXPECT_EQ(Tool(ball, true, {{2, 9, 0, 3}}).Frustums().size(), Tool(ball).Frustums().size());
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
    const std::vector<long> targets
        = {total / 2, total / 3, total - 5, 7, total / 2 + 3, total / 2 + 1};
    for (long target : targets) {
        WaitShown(sim, target, grid);
        // the same step simulated from the start, in a new simulation
        Simulation direct;
        direct.Start(tools, segments, xyz, triangles, 5);
        Grid directGrid;
        WaitShown(direct, target, directGrid);
        const std::vector<uint8_t> a = SampleInside(grid, points),
                                   b = SampleInside(directGrid, points);
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

    // every directed edge has its reverse: closed and oriented
    double volume = 0;
    EXPECT_EQ(OpenEdges(mesher, &volume), 0);
    // less than the box, by more than the slot of the flat end mill alone
    EXPECT_LT(volume, 20.0 * 20 * 6 - 12 * 4 * 2 + 1);
    EXPECT_GT(volume, 20.0 * 20 * 6 - 400);
}

// The mesh updated with the leaves that changed, going forwards and backwards on the timeline,
// stays closed
TEST(MaterialMesher, partialUpdatesAreClosed)
{
    ThreadPool pool(4);
    std::vector<std::shared_ptr<const Tool>> tools = {
        std::make_shared<Tool>(FlatProfile(1.5f)),
        std::make_shared<Tool>(BallProfile(1)),
    };
    std::vector<Segment> segments;
    std::mt19937 rng(3);
    std::uniform_real_distribution<float> U(2, 18);
    float at[3] = {10, 10, -1};
    for (int i = 0; i < 40; i++) {
        const float next[3] = {U(rng), U(rng), -1 - (float)(i % 4)};
        segments.push_back(Line(i % 2, at, next, 5));
        std::copy(next, next + 3, at);
    }
    const double lo[3] = {0, 0, -6}, hi[3] = {20, 20, 0};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    Simulation sim;
    sim.Start(tools, segments, xyz, triangles, 6);
    const long total = sim.TotalSteps();
    Grid grid;
    WaitShown(sim, 0, grid);
    Mesher mesher;
    mesher.Reset(sim.GetLayout(), pool);
    mesher.Update(grid, {}, pool);
    for (long step : {total / 4, total / 2, total, total / 3, total / 8 + 1, total}) {
        sim.SetTarget(step);
        std::vector<int> changed;
        for (int i = 0; i < 20000; i++) {
            if (!sim.Pending() && sim.TakeDisplay(grid, changed) == step) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        mesher.Update(grid, changed, pool);
        EXPECT_EQ(OpenEdges(mesher), 0) << "at step " << step;
    }
}

// Faces of thin walls get the thickness of the material behind them
TEST(MaterialMesher, thinWallsAreMeasured)
{
    ThreadPool pool(4);
    std::vector<std::shared_ptr<const Tool>> tools = {std::make_shared<Tool>(FlatProfile(2))};
    // two slots along Y leave a wall from x = 9 to 9.6 between them
    const float a[3] = {7, 4, -4}, b[3] = {7, 16, -4}, c[3] = {11.6f, 4, -4}, d[3] = {11.6f, 16, -4};
    std::vector<Segment> segments = {Line(0, a, b), Line(0, c, d)};
    const double lo[3] = {0, 0, -6}, hi[3] = {20, 20, 0};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    Simulation sim;
    sim.Start(tools, segments, xyz, triangles, 8);
    Grid grid;
    WaitShown(sim, sim.TotalSteps(), grid);
    Mesher mesher;
    mesher.SetMeasureThickness(true);
    mesher.Reset(sim.GetLayout(), pool);
    mesher.Update(grid, {}, pool);

    int wall = 0, floor = 0;
    ForCutVertices(mesher, [&](const MeshVertex& v) {
        // the faces of the wall, away from its ends and its foot
        const bool onWall = (std::abs(v.x - 9) < 1e-3 || std::abs(v.x - 9.6f) < 1e-3) && v.y > 6
            && v.y < 14 && v.z > -3.5f && v.z < -0.5f;
        if (onWall) {
            wall++;
            EXPECT_NEAR(DecodeThickness(v.thickness), 0.6f, 0.02f) << v.x << " " << v.y << " " << v.z;
        }
        // the floors of the slots, with the stock below them
        if (std::abs(v.z + 4) < 1e-3 && v.y > 6 && v.y < 14 && std::abs(v.x - 7) < 1) {
            floor++;
            EXPECT_NEAR(DecodeThickness(v.thickness), 2, 0.02f);
        }
    });
    EXPECT_GT(wall, 4);
    EXPECT_GT(floor, 2);
}

// Cut faces have the tag of the operation that cut them, the faces of the stock none
TEST(MaterialMesher, cutFacesKeepTheirOperation)
{
    ThreadPool pool(4);
    std::vector<std::shared_ptr<const Tool>> tools = {std::make_shared<Tool>(FlatProfile(2))};
    const float a[3] = {5, 4, -2}, b[3] = {5, 16, -2}, c[3] = {14, 4, -3}, d[3] = {14, 16, -3};
    std::vector<Segment> segments = {Line(0, a, b), Line(0, c, d)};
    segments[0].operation = 0;
    segments[1].operation = 3;
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
    mesher.Update(grid, {}, pool);

    int first = 0, second = 0;
    ForCutVertices(mesher, [&](const MeshVertex& v) {
        if (std::abs(v.z + 2) < 1e-3 && std::abs(v.x - 5) < 1 && v.y > 5 && v.y < 15) {
            first++;
            EXPECT_EQ(v.tag, OperationTag(0));
        }
        if (std::abs(v.z + 3) < 1e-3 && std::abs(v.x - 14) < 1 && v.y > 5 && v.y < 15) {
            second++;
            EXPECT_EQ(v.tag, OperationTag(3));
        }
    });
    EXPECT_GT(first, 0);
    EXPECT_GT(second, 0);
    // the faces of the stock are on the faces of the box; the floors of the slots are on grid
    // planes, where the material only touched by the tool along X and Y is still cut
    int inside = 0;
    for (int u = 0; u < mesher.UnitCount(); u++) {
        const UnitMesh& mesh = mesher.Mesh(u);
        for (uint32_t i : mesh.stock) {
            const MeshVertex& v = mesh.vertices[i];
            EXPECT_EQ(v.tag, 0);
            inside += v.x > lo[0] + 1e-3 && v.x < hi[0] - 1e-3 && v.y > lo[1] + 1e-3
                && v.y < hi[1] - 1e-3 && v.z > lo[2] + 1e-3 && v.z < hi[2] - 1e-3;
        }
    }
    EXPECT_EQ(inside, 0);
}

// The shading normals follow the faces: at the sharp edges of a pocket and of a V groove (made
// by the sharp tip of the tool, without normals), and the tip of a V groove narrower than the cells
// is in place all along it, across the grid
TEST(MaterialMesher, normalsAndCreasesFollowTheFaces)
{
    ThreadPool pool(4);
    std::vector<std::shared_ptr<const Tool>> tools = {
        std::make_shared<Tool>(FlatProfile(2)),
        std::make_shared<Tool>(VBitProfile(3, 45)),
    };
    const float a[3] = {4, 4, -2}, b[3] = {8, 4, -2};
    // a groove 0.8 deep, diagonal to the grid
    const float c[3] = {3, 8, -0.8f}, d[3] = {15, 20, -0.8f};
    std::vector<Segment> segments = {Line(0, a, b), Line(1, c, d)};
    const double lo[3] = {0, 0, -6}, hi[3] = {20, 24, 0};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    Simulation sim;
    sim.Start(tools, segments, xyz, triangles, 10);
    Grid grid;
    WaitShown(sim, sim.TotalSteps(), grid);
    Mesher mesher;
    mesher.Reset(sim.GetLayout(), pool);
    mesher.Update(grid, {}, pool);

    double area = 0, offArea = 0, deepest = 0;
    for (int u = 0; u < mesher.UnitCount(); u++) {
        const UnitMesh& mesh = mesher.Mesh(u);
        for (const auto* list : {&mesh.stock, &mesh.cut}) {
            for (size_t t = 0; t + 2 < list->size(); t += 3) {
                const MeshVertex* v[3];
                for (int k = 0; k < 3; k++) {
                    v[k] = &mesh.vertices[(*list)[t + k]];
                }
                const double e1[3] = {v[1]->x - v[0]->x, v[1]->y - v[0]->y, v[1]->z - v[0]->z};
                const double e2[3] = {v[2]->x - v[0]->x, v[2]->y - v[0]->y, v[2]->z - v[0]->z};
                double n[3] = {
                    e1[1] * e2[2] - e1[2] * e2[1],
                    e1[2] * e2[0] - e1[0] * e2[2],
                    e1[0] * e2[1] - e1[1] * e2[0],
                };
                const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
                if (len < 1e-9) {
                    continue;
                }
                area += len / 2;
                for (int k = 0; k < 3; k++) {
                    const double dot = (n[0] * v[k]->nx + n[1] * v[k]->ny + n[2] * v[k]->nz) / len
                        / 127;
                    if (dot < std::cos(30 * M_PI / 180)) {
                        offArea += len / 2;
                        break;
                    }
                }
                // the vertices along the middle of the groove
                for (int k = 0; k < 3; k++) {
                    const double along = (v[k]->x - 3 + v[k]->y - 8) / 2;
                    const double across = (v[k]->x - 3 - (v[k]->y - 8)) / std::sqrt(2.0);
                    if (along > 3 && along < 9 && std::abs(across) < 0.3 && v[k]->z > -1.5f) {
                        deepest = std::min(deepest, (double)v[k]->z);
                    }
                }
            }
        }
    }
    EXPECT_LT(offArea, 0.002 * area);
    EXPECT_NEAR(deepest, -0.8, 0.03);
    // the bottom of the groove all along it: no vertex near its tip stays far above it
    std::map<int, double> bottomBySlice;
    for (int u = 0; u < mesher.UnitCount(); u++) {
        for (const MeshVertex& v : mesher.Mesh(u).vertices) {
            const double along = (v.x - 3 + v.y - 8) / 2;
            const double across = (v.x - 3 - (v.y - 8)) / std::sqrt(2.0);
            if (along > 3 && along < 9 && std::abs(across) < 0.3 && v.z > -1.5f) {
                const int slice = (int)std::floor(along / 0.5);
                auto it = bottomBySlice.emplace(slice, 0.0).first;
                it->second = std::min(it->second, (double)v.z);
            }
        }
    }
    EXPECT_GE(bottomBySlice.size(), 10u);
    for (const auto& [slice, z] : bottomBySlice) {
        EXPECT_NEAR(z, -0.8, 0.05) << "at slice " << slice;
    }
}

// Signed distance from a model: a box, outside positive, inside negative, at its faces, edges and
// corners, and points beyond the range by their side
TEST(MaterialModel, signedDistanceFromABox)
{
    const double lo[3] = {0, 0, 0}, hi[3] = {10, 6, 4};
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(lo, hi, xyz, triangles);
    const ModelDistance model(xyz, triangles, 1.0f);
    struct Case
    {
        float p[3];
        float distance;
    };
    const Case cases[] = {
        {{5, 3, 4.2f}, 0.2f},                           // above the top
        {{5, 3, 3.7f}, -0.3f},                          // below it, inside
        {{-0.25f, 3, 2}, 0.25f},                        // beside a side
        {{10.3f, 6.4f, 2}, 0.5f},                       // beside an edge
        {{9.9f, 5.8f, 2}, -0.1f},                       // inside near an edge
        {{10.2f, 6.2f, 4.2f}, std::sqrt(3.0f) * 0.2f},  // beyond a corner
        {{5, 3, 6}, 1.0f},                              // above, beyond the range
        {{5, 3, 2}, -1.0f},                             // deep inside, beyond the range
    };
    for (const Case& c : cases) {
        EXPECT_NEAR(model.SignedDistance(c.p), c.distance, 1e-4)
            << c.p[0] << " " << c.p[1] << " " << c.p[2];
    }
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
