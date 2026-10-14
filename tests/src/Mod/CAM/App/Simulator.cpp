// SPDX-License-Identifier: LGPL-2.1-or-later

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/CutterProfile.h>
#include <Mod/CAM/PathSimulator/AppGL/DexelMesher.h>
#include <Mod/CAM/PathSimulator/AppGL/DexelSimulation.h>
#include <Mod/CAM/PathSimulator/AppGL/DexelStock.h>
#include <Mod/CAM/PathSimulator/AppGL/ThreadPool.h>

// NOLINTBEGIN(cppcoreguidelines-*,readability-*)

using namespace CAMSimulator;

namespace
{

constexpr float ToolLength = 40;

std::vector<float> FlatProfile(float radius)
{
    return {radius, ToolLength, radius, 0, 0, 0};
}

std::vector<float> BallProfile(float radius, int segments = 64)
{
    std::vector<float> profile = {radius, ToolLength, radius, radius};
    for (int i = 1; i <= segments; i++) {
        const float a = i * (float)M_PI / 2 / segments;
        profile.push_back(radius * std::cos(a));
        profile.push_back(radius - radius * std::sin(a));
    }
    return profile;
}

// neck radius 3 above height 6, widening to radius 7 at the tip
std::vector<float> DovetailProfile()
{
    return {3, ToolLength, 3, 6, 7, 0, 0, 0};
}

// box stock [0, size] x [0, size] x [-depth, 0], cell size 'cell'
void MakeBox(float size, float depth, std::vector<float>& xyz, std::vector<uint32_t>& triangles)
{
    xyz.clear();
    for (int i = 0; i < 8; i++) {
        xyz.push_back(i & 1 ? size : 0);
        xyz.push_back(i & 2 ? size : 0);
        xyz.push_back(i & 4 ? 0 : -depth);
    }
    triangles = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
}

struct Fixture
{
    ThreadPool pool {4};
    DexelStock stock;

    Fixture(float size = 20, float depth = 10, float cell = 0.25f)
    {
        std::vector<float> xyz;
        std::vector<uint32_t> triangles;
        MakeBox(size, depth, xyz, triangles);
        stock.Init(0, 0, cell, (int)std::lround(size / cell), (int)std::lround(size / cell));
        stock.SetMesh(xyz, triangles, pool);
    }

    // material intervals at the cell containing (x, y)
    std::vector<Interval> At(float x, float y) const
    {
        const Interval* iv;
        const int n = stock.Cell(
            (int)((x - stock.X0()) / stock.CellSize()),
            (int)((y - stock.Y0()) / stock.CellSize()),
            iv
        );
        return std::vector<Interval>(iv, iv + n);
    }
};

CutMove Move(const CutterProfile& tool, const float p0[3], const float p1[3], const float axis[3])
{
    CutMove move;
    move.tool = &tool;
    for (int k = 0; k < 3; k++) {
        move.p0[k] = p0[k];
        move.p1[k] = p1[k];
        move.a0[k] = axis[k];
        move.a1[k] = axis[k];
    }
    return move;
}

const float Up[3] = {0, 0, 1};

}  // namespace

TEST(CutterProfile, flatEndMill)
{
    CutterProfile tool(FlatProfile(3));
    EXPECT_FLOAT_EQ(tool.Radius(), 3);
    EXPECT_TRUE(tool.IsRadiallyMonotonic());
    const Interval* iv;
    ASSERT_EQ(tool.AxialIntervals(2.9f, iv), 1);
    EXPECT_FLOAT_EQ(iv[0].lo, 0);
    EXPECT_FLOAT_EQ(iv[0].hi, ToolLength);
    EXPECT_EQ(tool.AxialIntervals(3.1f, iv), 0);
}

TEST(CutterProfile, ballEndMill)
{
    CutterProfile tool(BallProfile(3));
    std::vector<Interval> iv;
    for (float d : {0.0f, 1.0f, 2.0f, 2.9f}) {
        tool.ComputeAxialIntervals(d, iv);
        ASSERT_EQ(iv.size(), 1u);
        EXPECT_NEAR(iv[0].lo, 3 - std::sqrt(9 - d * d), 0.01);
        EXPECT_FLOAT_EQ(iv[0].hi, ToolLength);
    }
}

TEST(CutterProfile, dovetailUndercut)
{
    CutterProfile tool(DovetailProfile());
    EXPECT_TRUE(tool.IsRadiallyMonotonic());
    std::vector<Interval> iv;
    // beyond the neck only the widening part near the tip
    tool.ComputeAxialIntervals(5, iv);
    ASSERT_EQ(iv.size(), 1u);
    EXPECT_FLOAT_EQ(iv[0].lo, 0);
    EXPECT_NEAR(iv[0].hi, 3, 1e-4);  // radius 5 at height 3
    tool.ComputeAxialIntervals(2, iv);
    ASSERT_EQ(iv.size(), 1u);
    EXPECT_FLOAT_EQ(iv[0].hi, ToolLength);
}

TEST(CutterProfile, lineIntervalsMatchSampling)
{
    // lines in random directions through every tool shape, compared with point sampling
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> coord(-8, 8), height(-2, 12);
    for (const auto& profile : {FlatProfile(3), BallProfile(3), DovetailProfile()}) {
        CutterProfile tool(profile);
        std::vector<Interval> iv;
        for (int i = 0; i < 200; i++) {
            float origin[3] = {coord(rng), coord(rng), height(rng)};
            float dir[3] = {coord(rng), coord(rng), coord(rng)};
            const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            for (float& d : dir) {
                d /= len;
            }
            tool.LineIntervals(origin, dir, iv);
            for (float s = -60; s <= 60; s += 0.0137f) {
                const float p[3]
                    = {origin[0] + s * dir[0], origin[1] + s * dir[1], origin[2] + s * dir[2]};
                const bool inside = tool.Contains(std::hypot(p[0], p[1]), p[2]);
                bool inInterval = false, nearBorder = false;
                for (const Interval& v : iv) {
                    inInterval |= s > v.lo && s < v.hi;
                    nearBorder |= std::abs(s - v.lo) < 1e-3 || std::abs(s - v.hi) < 1e-3;
                }
                if (!nearBorder) {
                    ASSERT_EQ(inside, inInterval) << "line " << i << " s " << s;
                }
            }
        }
    }
}

TEST(DexelStock, boxFromMesh)
{
    Fixture f;
    for (float x : {0.1f, 5.0f, 19.9f}) {
        for (float y : {0.1f, 12.0f, 19.9f}) {
            const auto iv = f.At(x, y);
            ASSERT_EQ(iv.size(), 1u);
            EXPECT_FLOAT_EQ(iv[0].lo, -10);
            EXPECT_FLOAT_EQ(iv[0].hi, 0);
        }
    }
}

TEST(DexelStock, boxDiagonalThroughCellCenters)
{
    // the triangle diagonals of the box faces pass exactly through cell centers, every dexel must
    // still get the full height
    ThreadPool pool(2);
    for (float size : {20.0f, 100.0f, 37.5f}) {
        for (float offset : {0.0f, 12.3f, -37.7f}) {
            for (int cells : {176, 138, 200}) {
                std::vector<float> xyz;
                std::vector<uint32_t> triangles;
                MakeBox(size, 10, xyz, triangles);
                for (size_t i = 0; i < xyz.size(); i += 3) {
                    xyz[i] += offset;
                    xyz[i + 1] += offset / 2;
                }
                DexelStock stock;
                const float cell = size / cells;
                stock.Init(offset, offset / 2, cell, cells, cells);
                stock.SetMesh(xyz, triangles, pool);
                int empty = 0;
                for (int iy = 0; iy < cells; iy++) {
                    for (int ix = 0; ix < cells; ix++) {
                        const Interval* iv;
                        if (stock.Cell(ix, iy, iv) != 1 || iv[0].lo != -10 || iv[0].hi != 0) {
                            empty++;
                        }
                    }
                }
                EXPECT_EQ(empty, 0) << "size " << size << " offset " << offset << " cells " << cells;
            }
        }
    }
}

TEST(DexelStock, horizontalSlot)
{
    Fixture f;
    CutterProfile tool(FlatProfile(2));
    const float p0[3] = {5, 10, -3}, p1[3] = {15, 10, -3};
    f.stock.Cut({Move(tool, p0, p1, Up)}, f.pool);

    auto iv = f.At(10, 10);  // in the slot
    ASSERT_EQ(iv.size(), 1u);
    EXPECT_FLOAT_EQ(iv[0].hi, -3);
    iv = f.At(10, 11.5f);  // in the slot, near its wall
    EXPECT_FLOAT_EQ(iv[0].hi, -3);
    iv = f.At(10, 12.5f);  // beyond the wall
    EXPECT_FLOAT_EQ(iv[0].hi, 0);
    iv = f.At(16.5f, 10);  // round end of the slot
    EXPECT_FLOAT_EQ(iv[0].hi, -3);
    iv = f.At(17.5f, 10);
    EXPECT_FLOAT_EQ(iv[0].hi, 0);
}

TEST(DexelStock, ballRamp)
{
    // the bottom of a ramp with a ball end mill matches the exact swept volume
    Fixture f;
    CutterProfile tool(BallProfile(3));
    const float p0[3] = {4, 10, -1}, p1[3] = {16, 10, -5};
    f.stock.Cut({Move(tool, p0, p1, Up)}, f.pool);
    for (float y : {10.0f, 11.0f, 12.0f}) {
        for (float x = 6; x < 14; x += 0.5f) {
            // dexel at the cell center
            const float cx = (std::floor(x / f.stock.CellSize()) + 0.5f) * f.stock.CellSize();
            const float cy = (std::floor(y / f.stock.CellSize()) + 0.5f) * f.stock.CellSize();
            float exact = 0;
            for (float t = 0; t <= 1; t += 1e-4f) {
                const float px = p0[0] + t * (p1[0] - p0[0]);
                const float pz = p0[2] + t * (p1[2] - p0[2]);
                const float d2 = (cx - px) * (cx - px) + (cy - 10) * (cy - 10);
                if (d2 < 9) {
                    exact = std::min(exact, pz + 3 - std::sqrt(9 - d2));
                }
            }
            const auto iv = f.At(x, y);
            ASSERT_EQ(iv.size(), 1u);
            EXPECT_NEAR(iv[0].hi, exact, 0.02) << "x " << x << " y " << y;
        }
    }
}

TEST(DexelStock, dovetailUndercut)
{
    // a dovetail moving along a wall leaves an overhang: two material intervals
    Fixture f;
    CutterProfile flat(FlatProfile(4));
    CutterProfile dovetail(DovetailProfile());
    // pocket x in [2, 12], floor at -8
    const float a[3] = {6, 0, -8}, b[3] = {6, 20, -8};
    const float c[3] = {8, 20, -8}, d[3] = {8, 0, -8};
    const float e[3] = {9, 10, -8}, g[3] = {9, 11, -8};
    f.stock.Cut({Move(flat, a, b, Up), Move(flat, c, d, Up)}, f.pool);
    // dovetail at x 9: bottom radius 7 reaches x 16, the neck (radius 3) stays inside the pocket
    f.stock.Cut({Move(dovetail, e, g, Up)}, f.pool);

    // dexel at x 14.625, 5.625 from the axis: the flank of the dovetail at height
    // (7 - 5.625) * 6 / 4 above its tip
    const auto iv = f.At(14.6f, 10.1f);
    ASSERT_EQ(iv.size(), 2u);
    EXPECT_FLOAT_EQ(iv[0].hi, -8);  // floor
    EXPECT_NEAR(iv[1].lo, -8 + 1.375f * 6 / 4, 0.01);
    EXPECT_FLOAT_EQ(iv[1].hi, 0);
}

TEST(DexelStock, tiltedToolMatchesVertical)
{
    // the general 5 axis path with an almost vertical axis gives the same result as the vertical
    // fast path
    Fixture vertical, tilted;
    CutterProfile tool(BallProfile(3));
    const float p0[3] = {5, 8, -2}, p1[3] = {15, 12, -4};
    const float almostUp[3] = {0, 1e-4f, 1};
    vertical.stock.Cut({Move(tool, p0, p1, Up)}, vertical.pool);
    tilted.stock.Cut({Move(tool, p0, p1, almostUp)}, tilted.pool);
    for (float x = 0.1f; x < 20; x += 0.7f) {
        for (float y = 0.1f; y < 20; y += 0.7f) {
            const auto a = vertical.At(x, y), b = tilted.At(x, y);
            ASSERT_EQ(a.size(), b.size()) << x << " " << y;
            for (size_t i = 0; i < a.size(); i++) {
                EXPECT_NEAR(a[i].hi, b[i].hi, vertical.stock.CellSize() * 0.3f) << x << " " << y;
            }
        }
    }
}

TEST(DexelStock, horizontalToolCutsSideways)
{
    // a flat end mill lying along +X cuts a cylinder of its radius around its axis
    Fixture f;
    CutterProfile tool(FlatProfile(2));
    const float tip[3] = {12, 10, -5};
    const float alongX[3] = {1, 0, 0};
    f.stock.Cut({Move(tool, tip, tip, alongX)}, f.pool);

    auto iv = f.At(15, 10);  // on the axis: a hole from -7 to -3
    ASSERT_EQ(iv.size(), 2u);
    EXPECT_NEAR(iv[0].hi, -7, 0.01);
    EXPECT_NEAR(iv[1].lo, -3, 0.01);
    iv = f.At(15, 11.5f);  // 1.5 (cell center 1.625) off the axis
    ASSERT_EQ(iv.size(), 2u);
    const float half = std::sqrt(4 - 1.625f * 1.625f);
    EXPECT_NEAR(iv[0].hi, -5 - half, 0.01);
    EXPECT_NEAR(iv[1].lo, -5 + half, 0.01);
    iv = f.At(11, 10);  // behind the tip
    ASSERT_EQ(iv.size(), 1u);
}

TEST(DexelSimulation, timelineMatchesDirectCut)
{
    // moving to positions on the timeline, forwards and backwards through snapshots, gives the
    // same stock as cutting directly
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(20, 10, xyz, triangles);

    std::vector<CutSegment> segments;
    for (int i = 0; i < 40; i++) {
        CutSegment s;
        s.arc = i % 3 == 0;
        const float y = 2 + (i % 16);
        const float z = -0.5f - 0.2f * (i / 4);
        if (s.arc) {
            s.center[0] = 10;
            s.center[1] = 10;
            s.radius = 2 + (i % 5);
            s.startAngle = 0.3f * i;
            s.sweep = 2.5f;
            s.PositionAt(0, s.start);
            s.start[2] = z;
            s.end[2] = z - 0.3f;
            s.PositionAt(1, s.end);
            s.end[2] = z - 0.3f;
        }
        else {
            const float start[3] = {2, y, z}, end[3] = {18, y + 1, z - 0.3f};
            std::copy(start, start + 3, s.start);
            std::copy(end, end + 3, s.end);
        }
        s.steps = 5 + i % 7;
        s.sweepSteps = s.steps;
        segments.push_back(s);
    }

    auto makeTools = [] {
        std::vector<std::unique_ptr<CutterProfile>> tools;
        tools.push_back(std::make_unique<CutterProfile>(BallProfile(1.5f)));
        return tools;
    };
    DexelSimulation sim;
    sim.Start(makeTools(), segments, xyz, triangles, 0.2f);
    DexelSimulation reference;
    reference.Start(makeTools(), segments, xyz, triangles, 0.2f);

    ThreadPool pool(4);
    for (long target :
         {sim.TotalSteps() / 2, sim.TotalSteps(), sim.TotalSteps() / 3, 7L, sim.TotalSteps()}) {
        sim.SetTarget(target);
        while (sim.Update(1000)) {
        }
        ASSERT_FALSE(sim.Pending());

        DexelStock direct;
        const DexelStock& initial = reference.Initial();
        direct.Init(initial.X0(), initial.Y0(), initial.CellSize(), initial.NX(), initial.NY());
        direct.Restore(initial.Tiles());
        std::vector<CutMove> moves;
        reference.AppendMoves(0, target, moves);
        direct.Cut(moves, pool);

        for (int iy = 0; iy < initial.NY(); iy++) {
            for (int ix = 0; ix < initial.NX(); ix++) {
                const Interval *a, *b;
                const int na = sim.Stock().Cell(ix, iy, a);
                const int nb = direct.Cell(ix, iy, b);
                ASSERT_EQ(na, nb) << "target " << target << " cell " << ix << " " << iy;
                for (int k = 0; k < na; k++) {
                    // pieces of moves cut in a different grouping can differ by a sliver
                    ASSERT_NEAR(a[k].lo, b[k].lo, 0.06f);
                    ASSERT_NEAR(a[k].hi, b[k].hi, 0.06f);
                }
            }
        }
    }
}

TEST(DexelMesher, faces)
{
    Fixture f(10, 5, 0.5f);
    DexelMesh mesh;
    BuildDexelMesh(f.stock, f.stock, 0, 0, f.stock.NX(), f.stock.NY(), mesh);
    // uncut box: top, bottom and 4 walls, each one rectangle, except the 2 columns at each end of
    // the walls, which are shaded towards the corner
    EXPECT_EQ(mesh.stockIndices.size(), (2u + 4 * (1 + 4)) * 6);
    EXPECT_TRUE(mesh.cutIndices.empty());

    DexelStock initial;
    initial.Init(0, 0, 0.5f, 20, 20);
    initial.Restore(f.stock.Tiles());
    f.stock.Freeze();
    CutterProfile tool(BallProfile(2));
    const float p0[3] = {2, 5, -1}, p1[3] = {8, 5, -1};
    f.stock.Cut({Move(tool, p0, p1, Up)}, f.pool);
    BuildDexelMesh(f.stock, initial, 0, 0, f.stock.NX(), f.stock.NY(), mesh);
    EXPECT_FALSE(mesh.cutIndices.empty());
    for (const DexelVertex& v : mesh.vertices) {
        EXPECT_GE(v.z, -5.0f);
        EXPECT_LE(v.z, 0.0f);
    }
}

// NOLINTEND(cppcoreguidelines-*,readability-*)
