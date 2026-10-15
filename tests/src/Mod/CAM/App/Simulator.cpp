// SPDX-License-Identifier: LGPL-2.1-or-later

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <Mod/CAM/PathSimulator/AppGL/CutterProfile.h>
#include <Mod/CAM/PathSimulator/AppGL/DexelLayout.h>
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

// Layout over the box stock of MakeBox, uniform with cells of 'cell', or with 'levels' finer
// levels in the regions
std::shared_ptr<DexelLayout> MakeLayout(
    float size,
    float depth,
    float cell,
    int levels = 0,
    const std::vector<LayoutRegion>& regions = {}
)
{
    const float lo[3] = {0, 0, -depth}, hi[3] = {size, size, 0};
    auto layout = std::make_shared<DexelLayout>();
    layout->Init(lo, hi, cell / (float)(1 << levels), levels);
    for (LayoutRegion region : regions) {
        region.UpdateBounds();
        layout->Refine(region, levels);
    }
    layout->Finish();
    return layout;
}

LayoutRegion VerticalRegion(float x0, float y0, float x1, float y1, float radius)
{
    LayoutRegion region {};
    region.a[0] = x0;
    region.a[1] = y0;
    region.b[0] = x1;
    region.b[1] = y1;
    region.radius = radius;
    region.vertical = true;
    region.zlo = -1e3f;
    region.zhi = 1e3f;
    return region;
}

struct Fixture
{
    ThreadPool pool {4};
    DexelStock stock;

    explicit Fixture(float size = 20, float depth = 10, float cell = 0.25f)
        : Fixture(MakeLayout(size, depth, cell), size, depth)
    {}
    Fixture(std::shared_ptr<DexelLayout> layout, float size, float depth)
    {
        std::vector<float> xyz;
        std::vector<uint32_t> triangles;
        MakeBox(size, depth, xyz, triangles);
        stock.Init(layout);
        stock.SetMesh(xyz, triangles, pool);
    }

    const DexelLayout& Layout() const
    {
        return stock.Layout();
    }

    // Grid coordinate of the grid line nearest to a world coordinate, on the grid of 'step'
    int Grid(int axis, float x, int step = 1) const
    {
        const int f = Layout().FineIndex(axis, x + Layout().FineCell() * step / 2);
        return f - ((f % step) + step) % step;
    }

    // Material along the whole line along 'axis' through the grid point nearest to (pu, pv), the
    // coordinates along the axes (axis + 1) % 3 and (axis + 2) % 3
    std::vector<Span> Material(int axis, float pu, float pv, int step = 1) const
    {
        const DexelLayout& layout = Layout();
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        int f[3];
        f[u] = Grid(u, pu, step);
        f[v] = Grid(v, pv, step);
        std::vector<Span> result;
        for (f[axis] = 0; f[axis] < layout.GridCells(axis);) {
            const int leaf = layout.FindLeaf(f[0], f[1], f[2]);
            const LayoutLeaf& l = layout.Leaf(leaf);
            const int i = (f[u] - l.origin[u]) / l.step, j = (f[v] - l.origin[v]) / l.step;
            EXPECT_EQ((f[u] - l.origin[u]) % l.step, 0);
            const Span* spans;
            Span scratch;
            const int n = stock.Line(leaf, axis * BrickAxisLines + j * BrickLines + i, spans, scratch);
            const float lo = layout.Coord(axis, l.origin[axis]);
            const float hi = layout.Coord(axis, l.origin[axis] + BrickCells * l.step);
            for (int k = 0; k < n; k++) {
                Span piece = spans[k];
                if (piece.hi < lo || piece.lo > hi) {
                    continue;
                }
                piece.lo = std::max(piece.lo, lo);
                piece.hi = std::min(piece.hi, hi);
                if (!result.empty() && result.back().hi >= piece.lo) {
                    result.back().hi = std::max(result.back().hi, piece.hi);
                    result.back().nhi = piece.nhi;
                }
                else {
                    result.push_back(piece);
                }
            }
            f[axis] = l.origin[axis] + BrickCells * l.step;
        }
        return result;
    }
    // Material along the Z line nearest to (x, y)
    std::vector<Span> At(float x, float y) const
    {
        return Material(2, x, y);
    }
    // World coordinate of the grid line nearest to a coordinate
    float Snap(int axis, float x) const
    {
        return Layout().Coord(axis, Grid(axis, x));
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

TEST(CutterProfile, smoothNormalsOfCoarseProfile)
{
    // a ball approximated by 10 edges, like the profiles of FreeCAD tools: the normals follow the
    // ball, not the edges, so the scallops it leaves are shaded like those of a real ball
    CutterProfile tool(BallProfile(3, 10));
    std::vector<Interval> intervals;
    std::vector<ProfileNormal> normals;
    for (float d = 0.05f; d < 2.9f; d += 0.05f) {
        tool.ComputeAxialIntervals(d, intervals, &normals);
        ASSERT_EQ(intervals.size(), 1u);
        // the bottom of the ball, normal away from its center at height 3
        EXPECT_NEAR(normals[0].r, d / 3, 0.02f) << "d " << d;
        EXPECT_LT(normals[0].z, 0);
    }
    for (float z = 0.1f; z < 2.9f; z += 0.1f) {
        ProfileNormal n;
        tool.SliceRadius(z, n);
        EXPECT_NEAR(n.z, (z - 3) / 3, 0.035f) << "z " << z;
    }
}

TEST(CutterProfile, sweptLowerEdge)
{
    // the cross section swept by a ball end mill is a disk around the ball center, with any axis;
    // a vertical flat end mill sweeps a flat bottom
    CutterProfile ball(BallProfile(1, 32));
    const float len = std::sqrt(0.3f * 0.3f + 0.4f * 0.4f + 1);
    const float tilted[3] = {0.3f / len, -0.4f / len, 1 / len};
    for (const float* axis : {Up, tilted}) {
        const auto edge = ball.SweptLowerEdge(axis, 4, 1025);
        const float cu = axis[1], cv = axis[2];  // the ball center
        for (float u = -0.9f; u <= 0.9f; u += 0.1f) {
            const float exact = cv - std::sqrt(1 - u * u);
            EXPECT_NEAR(edge.At(cu + u), exact, 0.01f) << "u " << u << " tilted " << (axis != Up);
        }
    }
    CutterProfile flat(FlatProfile(1));
    const auto edge = flat.SweptLowerEdge(Up, 4, 1025);
    for (float u = -0.95f; u <= 0.95f; u += 0.1f) {
        EXPECT_NEAR(edge.At(u), 0, 1e-4f);
    }
    EXPECT_FALSE(std::isfinite(edge.At(1.1f)));
}

TEST(CutterProfile, groovesOfToolShapes)
{
    // the angle at the ridge between passes beside each other, for tool shapes and axes
    auto groove = [](const CutterProfile& tool, const float axis[3], float spacing) {
        float angle, across, up;
        tool.SweptLowerEdge(axis, 4 * tool.Radius(), 2049).Groove(spacing, angle, across, up);
        return angle;
    };
    // a ball: twice the angle of the ball surface at half the spacing, for any axis
    CutterProfile ball(BallProfile(1, 16));
    const float tilted[3] = {0.3f, -0.4f, std::sqrt(1 - 0.25f)};
    EXPECT_NEAR(groove(ball, Up, 0.2f), 2 * std::asin(0.1f), 0.01f);
    EXPECT_NEAR(groove(ball, tilted, 0.2f), 2 * std::asin(0.1f), 0.01f);
    // a flat end mill leaves a flat floor, tilted it leaves grooves
    CutterProfile flat(FlatProfile(1));
    EXPECT_NEAR(groove(flat, Up, 0.5f), 0, 1e-3f);
    const float lead[3] = {std::sin(0.2f), 0, std::cos(0.2f)};
    const float side[3] = {0, std::sin(0.2f), std::cos(0.2f)};
    EXPECT_GT(groove(flat, lead, 1.0f), 0.15f);
    EXPECT_GT(groove(flat, side, 0.3f), 0.1f);
    // a V bit of 90 degrees: grooves of 90 degrees
    CutterProfile vbit({3, ToolLength, 3, 3, 0, 0});
    EXPECT_NEAR(groove(vbit, Up, 1), (float)M_PI / 2, 0.02f);
    // a bull nose: flat within the flat bottom, grooves beyond
    CutterProfile bull({2, ToolLength, 2, 0.5f, 1.5f + 0.5f * std::cos(0.4f), 0.5f - 0.5f * std::sin(0.4f),
                        1.5f + 0.5f * std::cos(0.8f), 0.5f - 0.5f * std::sin(0.8f), 1.5f + 0.5f * std::cos(1.2f),
                        0.5f - 0.5f * std::sin(1.2f), 1.5f, 0, 0, 0});
    EXPECT_NEAR(groove(bull, Up, 2), 0, 1e-3f);
    EXPECT_GT(groove(bull, Up, 3.6f), 0.2f);
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

TEST(CutterProfile, normalsAndSlices)
{
    CutterProfile ball(BallProfile(3));
    const Interval* iv;
    const ProfileNormal* normals;
    for (float d : {0.0f, 1.0f, 2.0f, 2.8f}) {
        ASSERT_EQ(ball.AxialIntervals(d, iv, normals), 1);
        // bottom: away from the center of the ball, top: up
        const float lo = iv[0].lo;
        EXPECT_NEAR(normals[0].r, d / 3, 0.05) << d;
        EXPECT_NEAR(normals[0].z, (lo - 3) / 3, 0.05) << d;
        EXPECT_NEAR(normals[1].r, 0, 1e-6);
        EXPECT_NEAR(normals[1].z, 1, 1e-6);
    }
    ProfileNormal n;
    for (float z : {0.5f, 1.5f, 2.9f}) {
        EXPECT_NEAR(ball.SliceRadius(z, n), std::sqrt(9 - (3 - z) * (3 - z)), 0.01) << z;
        EXPECT_NEAR(n.r, std::sqrt(9 - (3 - z) * (3 - z)) / 3, 0.05);
    }
    EXPECT_NEAR(ball.SliceRadius(10, n), 3, 1e-6);
    EXPECT_LT(ball.SliceRadius(-1, n), 0);
    EXPECT_NEAR(ball.MaxSliceRadius(0.5f, 10, n), 3, 1e-6);

    CutterProfile dovetail(DovetailProfile());
    EXPECT_NEAR(dovetail.SliceRadius(0, n), 7, 1e-5);
    EXPECT_NEAR(dovetail.SliceRadius(3, n), 5, 1e-5);
    EXPECT_NEAR(dovetail.MaxSliceRadius(2, 10, n), 7 - 2.0f * 4 / 6, 1e-5);
    EXPECT_GT(n.z, 0);  // the flank widens towards the tip, it faces up
}

TEST(CutterProfile, lineNormalsFaceOutwards)
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> coord(-8, 8), height(-2, 12);
    for (const auto& profile : {FlatProfile(3), BallProfile(3), DovetailProfile()}) {
        CutterProfile tool(profile);
        std::vector<Interval> iv;
        std::vector<float> normals;
        for (int i = 0; i < 200; i++) {
            float origin[3] = {coord(rng), coord(rng), height(rng)};
            float dir[3] = {coord(rng), coord(rng), coord(rng)};
            const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
            for (float& d : dir) {
                d /= len;
            }
            tool.LineIntervals(origin, dir, iv, &normals);
            ASSERT_EQ(normals.size(), 6 * iv.size());
            for (size_t k = 0; k < iv.size(); k++) {
                const float* nlo = &normals[6 * k];
                const float* nhi = &normals[6 * k + 3];
                EXPECT_LE(nlo[0] * dir[0] + nlo[1] * dir[1] + nlo[2] * dir[2], 1e-3f);
                EXPECT_GE(nhi[0] * dir[0] + nhi[1] * dir[1] + nhi[2] * dir[2], -1e-3f);
            }
        }
    }
}

TEST(DexelLayout, refinedAndBalanced)
{
    // finest bricks along a line, neighbors differ by at most one level
    auto layout = MakeLayout(40, 10, 1.0f, 3, {VerticalRegion(5, 5, 35, 20, 1)});
    EXPECT_EQ(layout->MaxLevel(), 3);
    int finest = 0;
    std::vector<int> units(layout->UnitCount(), 0);
    for (int i = 0; i < layout->LeafCount(); i++) {
        const LayoutLeaf& leaf = layout->Leaf(i);
        finest += leaf.level == 3;
        units[leaf.unit]++;
        EXPECT_EQ(layout->FindLeaf(leaf.origin[0], leaf.origin[1], leaf.origin[2]), i);
        const int size = BrickCells * leaf.step;
        for (int d = 0; d < 27; d++) {
            const int p[3] = {
                leaf.origin[0] + (d % 3 - 1) * (size / 2 + 1) + size / 2,
                leaf.origin[1] + (d / 3 % 3 - 1) * (size / 2 + 1) + size / 2,
                leaf.origin[2] + (d / 9 - 1) * (size / 2 + 1) + size / 2,
            };
            const int neighbor = layout->FindLeaf(p[0], p[1], p[2]);
            if (neighbor >= 0) {
                EXPECT_LE(std::abs(layout->Leaf(neighbor).level - leaf.level), 1);
            }
        }
    }
    EXPECT_GT(finest, 0);
    // every leaf in one unit, units are small
    int total = 0;
    for (int u = 0; u < layout->UnitCount(); u++) {
        EXPECT_EQ((int)layout->UnitLeaves(u).size(), units[u]);
        EXPECT_LE(units[u], 16);
        total += units[u];
    }
    EXPECT_EQ(total, layout->LeafCount());
    // the finest level is at the line, not far from it
    const int far = layout->FindLeaf(layout->FineIndex(0, 35), layout->FineIndex(1, 38), 2);
    EXPECT_LT(layout->Leaf(far).level, 3);
    const int near = layout->FindLeaf(layout->FineIndex(0, 20), layout->FineIndex(1, 12.5f), 2);
    EXPECT_EQ(layout->Leaf(near).level, 3);
}

TEST(DexelStock, boxFromMesh)
{
    Fixture f;
    for (float x : {0.4f, 5.0f, 19.6f}) {
        for (float y : {0.4f, 12.0f, 19.6f}) {
            const auto iv = f.At(x, y);
            ASSERT_EQ(iv.size(), 1u);
            EXPECT_FLOAT_EQ(iv[0].lo, -10);
            EXPECT_FLOAT_EQ(iv[0].hi, 0);
            float n[3];
            UnpackNormal(iv[0].nhi, n);
            EXPECT_NEAR(n[2], 1, 1e-4);
            UnpackNormal(iv[0].nlo, n);
            EXPECT_NEAR(n[2], -1, 1e-4);
            EXPECT_FALSE(IsCutFace(iv[0].nhi));
        }
    }
    for (float a : {0.4f, 7.0f, 19.6f}) {
        for (float z : {-9.6f, -5.0f, -0.4f}) {
            const auto x = f.Material(0, a, z);
            ASSERT_EQ(x.size(), 1u);
            EXPECT_FLOAT_EQ(x[0].lo, 0);
            EXPECT_FLOAT_EQ(x[0].hi, 20);
            float n[3];
            UnpackNormal(x[0].nhi, n);
            EXPECT_NEAR(n[0], 1, 1e-4);
            const auto y = f.Material(1, z, a);
            ASSERT_EQ(y.size(), 1u);
            EXPECT_FLOAT_EQ(y[0].lo, 0);
            EXPECT_FLOAT_EQ(y[0].hi, 20);
            UnpackNormal(y[0].nlo, n);
            EXPECT_NEAR(n[1], -1, 1e-4);
        }
    }
    // the bricks far inside are completely filled
    int solid = 0;
    for (int i = 0; i < f.Layout().LeafCount(); i++) {
        solid += f.stock.IsSolid(i);
    }
    EXPECT_GT(solid, 0);
}

TEST(DexelStock, boxDiagonalThroughGridPoints)
{
    // the triangle diagonals of the box faces may pass exactly through grid points, every line
    // must still get the full box
    ThreadPool pool(2);
    for (float size : {20.0f, 100.0f, 37.5f}) {
        for (float offset : {0.0f, 12.3f, -37.7f}) {
            for (int cells : {64, 80, 100}) {
                std::vector<float> xyz;
                std::vector<uint32_t> triangles;
                MakeBox(size, 10, xyz, triangles);
                for (size_t i = 0; i < xyz.size(); i += 3) {
                    xyz[i] += offset;
                    xyz[i + 1] += offset / 2;
                }
                const float lo[3] = {offset, offset / 2, -10};
                const float hi[3] = {offset + size, offset / 2 + size, 0};
                auto layout = std::make_shared<DexelLayout>();
                layout->Init(lo, hi, size / cells, 0);
                layout->Finish();
                DexelStock stock;
                stock.Init(layout);
                stock.SetMesh(xyz, triangles, pool);
                int wrong = 0, lines = 0;
                Span scratch;
                for (int leaf = 0; leaf < layout->LeafCount(); leaf++) {
                    const LayoutLeaf& l = layout->Leaf(leaf);
                    for (int j = 0; j < BrickLines; j++) {
                        for (int i = 0; i < BrickLines; i++) {
                            const float x = layout->Coord(0, l.origin[0] + i * l.step);
                            const float y = layout->Coord(1, l.origin[1] + j * l.step);
                            if (x <= lo[0] || x >= hi[0] || y <= lo[1] || y >= hi[1]) {
                                continue;
                            }
                            float tlo, thi;
                            stock.LineRange(leaf, 2, tlo, thi);
                            if (thi < -10 || tlo > 0) {
                                continue;
                            }
                            const Span* spans;
                            const int n = stock.Line(leaf, 2 * BrickAxisLines + j * BrickLines + i, spans, scratch);
                            lines++;
                            if (n != 1 || spans[0].lo != std::max(-10.0f, tlo)
                                || spans[0].hi != std::min(0.0f, thi)) {
                                wrong++;
                            }
                        }
                    }
                }
                EXPECT_GT(lines, 0);
                EXPECT_EQ(wrong, 0) << "size " << size << " offset " << offset << " cells " << cells;
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
    EXPECT_TRUE(IsCutFace(iv[0].nhi));
    iv = f.At(10, 11.5f);  // in the slot, near its wall
    EXPECT_FLOAT_EQ(iv[0].hi, -3);
    iv = f.At(10, 12.5f);  // beyond the wall
    EXPECT_FLOAT_EQ(iv[0].hi, 0);
    EXPECT_FALSE(IsCutFace(iv[0].nhi));
    iv = f.At(16.5f, 10);  // round end of the slot
    EXPECT_FLOAT_EQ(iv[0].hi, -3);
    iv = f.At(17.5f, 10);
    EXPECT_FLOAT_EQ(iv[0].hi, 0);

    // across the slot along X and Y, above its floor: exact walls
    const float y = f.Snap(1, 10);
    const float half = std::sqrt(4 - (y - 10) * (y - 10));
    auto x = f.Material(0, 10, -2);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(x[0].hi, 5 - half, 1e-4);
    EXPECT_NEAR(x[1].lo, 15 + half, 1e-4);
    float n[3];
    UnpackNormal(x[0].nhi, n);
    EXPECT_NEAR(n[0], 1, 0.01);
    EXPECT_TRUE(IsCutFace(x[0].nhi));
    auto across = f.Material(1, -2, 10);
    ASSERT_EQ(across.size(), 2u);
    EXPECT_NEAR(across[0].hi, 8, 1e-4);
    EXPECT_NEAR(across[1].lo, 12, 1e-4);
    UnpackNormal(across[1].nlo, n);
    EXPECT_NEAR(n[1], -1, 0.01);
    // below the floor the lines are not cut
    EXPECT_EQ(f.Material(0, 10, -3.5f).size(), 1u);
    // all lines across the straight part of the slot end at its walls
    for (float x = 5.05f; x < 15; x += 0.37f) {
        const auto line = f.Material(1, -2.2f, x);
        ASSERT_EQ(line.size(), 2u) << x;
        EXPECT_NEAR(line[0].hi, 8, 1e-4) << x;
        EXPECT_NEAR(line[1].lo, 12, 1e-4) << x;
    }
}

TEST(DexelStock, wallsOfLongMoves)
{
    // the lines across a long move end at its walls everywhere, also where they pass the path
    // of the tool center at its ends
    Fixture f(MakeLayout(50, 20, 0.1f), 50, 20);
    CutterProfile tool(FlatProfile(3));
    const float p0[3] = {10, 10, -6}, p1[3] = {40, 10, -6};
    f.stock.Cut({Move(tool, p0, p1, Up)}, f.pool);
    int checked = 0;
    for (float x = 10.02f; x < 40; x += 0.1f) {
        for (float z : {-0.3f, -2.2f, -5.5f}) {
            const auto line = f.Material(1, z, x);
            ASSERT_EQ(line.size(), 2u) << x << " " << z;
            EXPECT_NEAR(line[0].hi, 7, 1e-4) << x << " " << z;
            EXPECT_NEAR(line[1].lo, 13, 1e-4) << x << " " << z;
            checked++;
        }
    }
    EXPECT_GT(checked, 800);
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
            const float cx = f.Snap(0, x), cy = f.Snap(1, y);
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
            // the normal of the surface follows the ball
            float n[3];
            UnpackNormal(iv[0].nhi, n);
            EXPECT_GT(n[2], 0.3f);
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

    // the flank of the dovetail at the distance of the line from the axis
    const float dist = f.Snap(0, 14.6f) - 9;
    const auto iv = f.At(14.6f, 10.5f);
    ASSERT_EQ(iv.size(), 2u);
    EXPECT_FLOAT_EQ(iv[0].hi, -8);  // floor
    EXPECT_NEAR(iv[1].lo, -8 + (7 - dist) * 6 / 4, 0.01);
    EXPECT_FLOAT_EQ(iv[1].hi, 0);

    // along X under the overhang: the flank at radius 7 - 1 * 4 / 6 one above the tip
    const float z = f.Snap(2, -7);
    const auto x = f.Material(0, 10.5f, -7);
    ASSERT_EQ(x.size(), 2u);
    EXPECT_NEAR(x[0].hi, 2, 1e-3);
    EXPECT_NEAR(x[1].lo, 9 + 7 - (z + 8) * 4 / 6, 0.01);
    float n[3];
    UnpackNormal(x[1].nlo, n);
    EXPECT_LT(n[0], -0.5f);
    EXPECT_LT(n[2], -0.3f);  // the overhang faces down into the groove
}

TEST(DexelStock, tiltedToolMatchesVertical)
{
    // the general 5 axis path with an almost vertical axis gives the same result as the vertical
    // fast path, along all three axes
    Fixture vertical, tilted;
    CutterProfile tool(BallProfile(3));
    const float p0[3] = {5, 8, -2}, p1[3] = {15, 12, -4};
    const float almostUp[3] = {0, 1e-4f, 1};
    vertical.stock.Cut({Move(tool, p0, p1, Up)}, vertical.pool);
    tilted.stock.Cut({Move(tool, p0, p1, almostUp)}, tilted.pool);
    const float cell = vertical.Layout().FineCell();
    for (int axis = 0; axis < 3; axis++) {
        for (float u = 0.1f; u < 20; u += 0.7f) {
            for (float v = 0.1f; v < 20; v += 0.7f) {
                // lines along X and Y at heights in the stock
                const float pu = axis == 1 ? -u / 2 : u;
                const float pv = axis == 0 ? -v / 2 : v;
                const auto a = vertical.Material(axis, pu, pv), b = tilted.Material(axis, pu, pv);
                ASSERT_EQ(a.size(), b.size()) << axis << " " << pu << " " << pv;
                for (size_t i = 0; i < a.size(); i++) {
                    EXPECT_NEAR(a[i].lo, b[i].lo, cell * 0.3f) << axis << " " << pu << " " << pv;
                    EXPECT_NEAR(a[i].hi, b[i].hi, cell * 0.3f) << axis << " " << pu << " " << pv;
                }
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

    const float dy = f.Snap(1, 10) - 10;
    auto iv = f.At(15, 10);  // near the axis: a hole
    ASSERT_EQ(iv.size(), 2u);
    EXPECT_NEAR(iv[0].hi, -5 - std::sqrt(4 - dy * dy), 0.01);
    EXPECT_NEAR(iv[1].lo, -5 + std::sqrt(4 - dy * dy), 0.01);
    const float dy2 = f.Snap(1, 11.5f) - 10;
    iv = f.At(15, 11.5f);
    ASSERT_EQ(iv.size(), 2u);
    EXPECT_NEAR(iv[0].hi, -5 - std::sqrt(4 - dy2 * dy2), 0.01);
    iv = f.At(11, 10);  // behind the tip
    ASSERT_EQ(iv.size(), 1u);
    // along the axis: the flat tip of the tool
    const auto x = f.Material(0, 10, -5);
    ASSERT_EQ(x.size(), 1u);
    EXPECT_NEAR(x[0].hi, 12, 1e-3);
    float n[3];
    UnpackNormal(x[0].nhi, n);
    EXPECT_NEAR(n[0], 1, 0.01);
}

TEST(DexelStock, adaptiveLevelsAgree)
{
    // lines shared by bricks of different levels get the same material
    const float size = 20, depth = 10;
    auto layout = MakeLayout(size, depth, 1.0f, 2, {VerticalRegion(4, 4, 16, 16, 1)});
    Fixture f(layout, size, depth);
    CutterProfile tool(BallProfile(2));
    const float p0[3] = {3, 6, -1}, p1[3] = {17, 14, -3};
    f.stock.Cut({Move(tool, p0, p1, Up)}, f.pool);
    // every grid point of a coarse leaf on its border is also a point of its finer neighbor:
    // compare the inside state of the Z lines through it
    const DexelLayout& l = f.Layout();
    int compared = 0;
    Span scratch;
    for (int leaf = 0; leaf < l.LeafCount(); leaf++) {
        const LayoutLeaf& a = l.Leaf(leaf);
        for (int j = 0; j < BrickLines; j++) {
            for (int i = 0; i < BrickLines; i++) {
                for (int k = 0; k < BrickLines; k++) {
                    const int p[3] = {a.origin[0] + i * a.step, a.origin[1] + j * a.step, a.origin[2] + k * a.step};
                    const int other = l.FindLeaf(p[0], p[1], p[2]);
                    if (other < 0 || other == leaf) {
                        continue;
                    }
                    const LayoutLeaf& b = l.Leaf(other);
                    if ((p[0] - b.origin[0]) % b.step || (p[1] - b.origin[1]) % b.step
                        || (p[2] - b.origin[2]) % b.step) {
                        continue;
                    }
                    auto inside = [&](int which, const LayoutLeaf& w) {
                        const int li = (p[0] - w.origin[0]) / w.step, lj = (p[1] - w.origin[1]) / w.step;
                        const Span* spans;
                        const int n = f.stock.Line(which, 2 * BrickAxisLines + lj * BrickLines + li, spans, scratch);
                        const float z = l.Coord(2, p[2]);
                        for (int s = 0; s < n; s++) {
                            if (spans[s].lo <= z && z < spans[s].hi) {
                                return true;
                            }
                        }
                        return false;
                    };
                    ASSERT_EQ(inside(leaf, a), inside(other, b)) << leaf << " " << other;
                    compared++;
                }
            }
        }
    }
    EXPECT_GT(compared, 1000);
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
        tools.push_back(std::make_unique<CutterProfile>(BallProfile(0.75f)));
        return tools;
    };
    DexelSimulation sim;
    sim.Start(makeTools(), segments, xyz, triangles, 10);
    DexelSimulation reference;
    reference.Start(makeTools(), segments, xyz, triangles, 10);
    EXPECT_GT(sim.Stock().Layout().MaxLevel(), 0);

    ThreadPool pool(4);
    Span sa, sb;
    for (long target :
         {sim.TotalSteps() / 2, sim.TotalSteps(), sim.TotalSteps() / 3, 7L, sim.TotalSteps()}) {
        sim.SetTarget(target);
        while (sim.Pending()) {
            sim.Update(1000);
        }
        ASSERT_FALSE(sim.Pending());

        DexelStock direct;
        direct.Init(reference.Stock().LayoutRef());
        direct.Restore(reference.Initial().Bricks());
        direct.SetTopZ(reference.Initial().TopZ());
        std::vector<CutMove> moves;
        reference.AppendMoves(0, target, moves);
        direct.Cut(moves, pool);

        // arcs are divided into different chords when cut in other groups, a line along a
        // surface can then be just inside the material in one and outside in the other: compare
        // the material on every line, a few lines may differ
        long lines = 0, different = 0;
        for (int leaf = 0; leaf < direct.Layout().LeafCount(); leaf++) {
            for (int line = 0; line < BrickLineCount; line++) {
                const Span *a, *b;
                const int na = sim.Stock().Line(leaf, line, a, sa);
                const int nb = direct.Line(leaf, line, b, sb);
                float la = 0, lb = 0;
                for (int k = 0; k < na; k++) {
                    la += a[k].hi - a[k].lo;
                }
                for (int k = 0; k < nb; k++) {
                    lb += b[k].hi - b[k].lo;
                }
                lines += na > 0 || nb > 0;
                different += std::abs(la - lb) > 0.05f;
            }
        }
        EXPECT_GT(lines, 10000);
        EXPECT_LT(different, lines / 1000) << "target " << target;
    }
}

// Every directed edge of the triangles has its reverse: a closed surface without cracks
void ExpectClosed(const DexelMesh& mesh, const std::vector<uint32_t>& indices)
{
    std::map<std::tuple<float, float, float>, int> ids;
    auto id = [&](uint32_t index) {
        const DexelVertex& v = mesh.vertices[index];
        return ids.emplace(std::make_tuple(v.x, v.y, v.z), (int)ids.size()).first->second;
    };
    std::map<std::pair<int, int>, int> edges;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const int v[3] = {id(indices[t]), id(indices[t + 1]), id(indices[t + 2])};
        for (int k = 0; k < 3; k++) {
            edges[{v[k], v[(k + 1) % 3]}]++;
        }
    }
    int open = 0;
    for (const auto& [edge, count] : edges) {
        const auto reverse = edges.find({edge.second, edge.first});
        open += reverse == edges.end() || reverse->second != count;
    }
    EXPECT_EQ(open, 0);
}

std::vector<uint32_t> AllIndices(const DexelMesh& mesh)
{
    std::vector<uint32_t> all = mesh.stockIndices;
    all.insert(all.end(), mesh.cutIndices.begin(), mesh.cutIndices.end());
    return all;
}

double Volume(const DexelMesh& mesh, const std::vector<uint32_t>& indices)
{
    double volume = 0;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const DexelVertex& a = mesh.vertices[indices[t]];
        const DexelVertex& b = mesh.vertices[indices[t + 1]];
        const DexelVertex& c = mesh.vertices[indices[t + 2]];
        volume += (a.x * ((double)b.y * c.z - (double)b.z * c.y) - a.y * ((double)b.x * c.z - (double)b.z * c.x)
                   + a.z * ((double)b.x * c.y - (double)b.y * c.x))
            / 6;
    }
    return volume;
}

// Mesh of all units of the layout, built separately
DexelMesh MeshUnits(const DexelStock& stock)
{
    DexelMesh all, part;
    for (int unit = 0; unit < stock.Layout().UnitCount(); unit++) {
        BuildDexelMesh(stock, stock.Layout().UnitLeaves(unit), part);
        const auto base = (uint32_t)all.vertices.size();
        all.vertices.insert(all.vertices.end(), part.vertices.begin(), part.vertices.end());
        for (uint32_t i : part.stockIndices) {
            all.stockIndices.push_back(base + i);
        }
        for (uint32_t i : part.cutIndices) {
            all.cutIndices.push_back(base + i);
        }
    }
    return all;
}

TEST(DexelMesher, box)
{
    Fixture f(10, 5, 0.5f);
    const DexelMesh mesh = MeshUnits(f.stock);
    EXPECT_TRUE(mesh.cutIndices.empty());
    ExpectClosed(mesh, mesh.stockIndices);
    // sharp edges and corners: the exact volume, vertices on the box
    EXPECT_NEAR(Volume(mesh, mesh.stockIndices), 500, 1e-2);
    for (const DexelVertex& v : mesh.vertices) {
        EXPECT_GE(v.x, -1e-4f);
        EXPECT_LE(v.x, 10 + 1e-4f);
        EXPECT_GE(v.z, -5 - 1e-4f);
        EXPECT_LE(v.z, 1e-4f);
        // flat faces of the box
        const int axis = std::abs(v.nx) > 120 ? 0 : std::abs(v.ny) > 120 ? 1 : 2;
        EXPECT_GT(std::abs(axis == 0 ? v.nx : axis == 1 ? v.ny : v.nz), 120);
    }
}

TEST(DexelMesher, cutsAcrossLevelsAreClosed)
{
    const float size = 20, depth = 10;
    auto layout = MakeLayout(size, depth, 1.0f, 2, {VerticalRegion(4, 4, 16, 16, 2)});
    Fixture f(layout, size, depth);
    CutterProfile ball(BallProfile(2));
    CutterProfile flat(FlatProfile(3));
    const float p0[3] = {2, 3, -1.3f}, p1[3] = {18, 17, -3.1f};
    const float q0[3] = {3, 15, -2.2f}, q1[3] = {16, 6, -2.2f};
    f.stock.Cut({Move(ball, p0, p1, Up), Move(flat, q0, q1, Up)}, f.pool);
    const DexelMesh mesh = MeshUnits(f.stock);
    EXPECT_FALSE(mesh.cutIndices.empty());
    const auto all = AllIndices(mesh);
    ExpectClosed(mesh, all);
    const double volume = Volume(mesh, all);
    EXPECT_LT(volume, size * size * depth - 100);
    EXPECT_GT(volume, size * size * depth - 400);
}

TEST(DexelMesher, tiltedCutIsClosed)
{
    Fixture f(20, 10, 0.5f);
    CutterProfile ball(BallProfile(2));
    const float p0[3] = {3, 3, -2}, p1[3] = {17, 15, -1};
    const float a0[3] = {0.3f, 0, 0.954f};
    CutMove move = Move(ball, p0, p1, a0);
    move.a1[0] = 0;
    move.a1[1] = 0.5f;
    move.a1[2] = 0.866f;
    f.stock.Cut({move}, f.pool);
    const DexelMesh mesh = MeshUnits(f.stock);
    EXPECT_FALSE(mesh.cutIndices.empty());
    ExpectClosed(mesh, AllIndices(mesh));
}

TEST(DexelMesher, partialUpdatesAreClosed)
{
    // the mesh is closed after every update, also when only part of the changes is done: cuts in
    // separate places are updated one after the other
    const float size = 20, depth = 10;
    auto layout = MakeLayout(size, depth, 1.0f, 2, {VerticalRegion(2, 2, 18, 18, 2)});
    Fixture f(layout, size, depth);
    DexelMeshUnits units;
    units.Reset(f.stock);
    units.Update(f.stock, f.pool, 1e9);
    ASSERT_FALSE(units.Pending());
    CutterProfile ball(BallProfile(2));
    int partial = 0, updates = 0;
    for (int pass = 0; pass < 3; pass++) {
        const float y = 3 + 1.5f * pass;
        const float p0[3] = {2, y, -1.5f - 0.3f * pass}, p1[3] = {8, y + 1, -2.0f - 0.3f * pass};
        const float q0[3] = {12, 19 - y, -1.5f}, q1[3] = {18, 18 - y, -2.5f};
        f.stock.Cut({Move(ball, p0, p1, Up), Move(ball, q0, q1, Up)}, f.pool);
        do {
            units.Update(f.stock, f.pool, 0);
            partial += units.Pending();
            if (updates++ % 3 != 0 && units.Pending()) {
                continue;
            }
            DexelMesh all;
            for (int unit = 0; unit < layout->UnitCount(); unit++) {
                const DexelMesh& part = units.Mesh(unit);
                const auto base = (uint32_t)all.vertices.size();
                all.vertices.insert(all.vertices.end(), part.vertices.begin(), part.vertices.end());
                for (uint32_t i : part.stockIndices) {
                    all.stockIndices.push_back(base + i);
                }
                for (uint32_t i : part.cutIndices) {
                    all.stockIndices.push_back(base + i);
                }
            }
            ExpectClosed(all, all.stockIndices);
        } while (units.Pending());
    }
    EXPECT_GT(partial, 0);
}

TEST(DexelMesher, ballScallopsAreKept)
{
    // parallel passes of a ball end mill leave scallops 0.042 high, less than the tolerance of
    // merged cubes, but their normals differ by up to 19 degrees: the mesh keeps both
    Fixture f(20, 10, 0.25f);
    const float radius = 3, stepover = 1;
    CutterProfile ball(BallProfile(radius));
    std::vector<CutMove> moves;
    for (float y = 2; y < 18; y += stepover) {
        const float p0[3] = {1, y, -1}, p1[3] = {19, y, -1};
        moves.push_back(Move(ball, p0, p1, Up));
    }
    f.stock.Cut(moves, f.pool);
    const DexelMesh mesh = MeshUnits(f.stock);
    float zlo = 1e9f, zhi = -1e9f;
    int maxNy = 0;
    for (uint32_t i : mesh.cutIndices) {
        const DexelVertex& v = mesh.vertices[i];
        if (v.x < 6 || v.x > 14 || v.y < 6 || v.y > 14 || v.nz < 60) {
            continue;
        }
        zlo = std::min(zlo, v.z);
        zhi = std::max(zhi, v.z);
        maxNy = std::max(maxNy, std::abs((int)v.ny));
    }
    EXPECT_GT(zhi - zlo, 0.03f);
    EXPECT_GT(maxNy, 0.05f * 127);
}

TEST(DexelMesher, ridgesBetweenPassesAreSharp)
{
    // the ridges between passes of a ball end mill: vertices next to a ridge take the normals of
    // their side, the shading is not averaged across the ridge
    Fixture f(20, 10, 0.25f);
    const float radius = 3, stepover = 1;
    CutterProfile ball(BallProfile(radius, 10));
    std::vector<CutMove> moves;
    std::vector<float> centers;
    for (float y = 2; y < 18; y += stepover) {
        const float p0[3] = {1, y, -1}, p1[3] = {19, y, -1};
        moves.push_back(Move(ball, p0, p1, Up));
        centers.push_back(y);
    }
    f.stock.Cut(moves, f.pool);
    const DexelMesh mesh = MeshUnits(f.stock);
    double meshSum = 0, exactSum = 0;
    for (uint32_t i : mesh.cutIndices) {
        const DexelVertex& v = mesh.vertices[i];
        if (v.x < 6 || v.x > 14 || v.y < 6 || v.y > 14 || v.nz < 60) {
            continue;
        }
        float center = 0;
        for (float c : centers) {
            if (std::abs(v.y - c) < std::abs(v.y - center)) {
                center = c;
            }
        }
        meshSum += std::abs(v.ny / 127.0);
        exactSum += std::abs(center - v.y) / radius;
    }
    // averaged across the ridges, about half
    EXPECT_GT(meshSum, 0.8 * exactSum);
}

TEST(DexelMesher, smoothConvexWallHasNoRidges)
{
    // the wall of a round boss bends convex about the same everywhere: no ridges, the normals
    // follow the wall
    Fixture f(20, 10, 0.25f);
    CutterProfile flat(FlatProfile(2));
    const float boss = 1.5f;
    std::vector<CutMove> moves;
    const int sides = 360;
    for (int s = 0; s < sides; s++) {
        const float a0 = s * 2 * (float)M_PI / sides, a1 = (s + 1) * 2 * (float)M_PI / sides;
        const float r = boss + 2;
        const float p0[3] = {10 + r * std::cos(a0), 10 + r * std::sin(a0), -3};
        const float p1[3] = {10 + r * std::cos(a1), 10 + r * std::sin(a1), -3};
        moves.push_back(Move(flat, p0, p1, Up));
    }
    f.stock.Cut(moves, f.pool);
    const DexelMesh mesh = MeshUnits(f.stock);
    int count = 0;
    for (uint32_t i : mesh.cutIndices) {
        const DexelVertex& v = mesh.vertices[i];
        const float dx = v.x - 10, dy = v.y - 10, len = std::hypot(dx, dy);
        if (v.z > -0.5f || v.z < -2.5f || std::abs(len - boss) > 0.3f) {
            continue;
        }
        const float n = std::sqrt((float)(v.nx * v.nx + v.ny * v.ny + v.nz * v.nz));
        EXPECT_GT((v.nx * dx + v.ny * dy) / (n * len), std::cos(3 * (float)M_PI / 180));
        count++;
    }
    EXPECT_GT(count, 100);
}

namespace
{

CutSegment Line(int tool, const float p0[3], const float p1[3])
{
    CutSegment s;
    s.tool = tool;
    std::copy(p0, p0 + 3, s.start);
    std::copy(p1, p1 + 3, s.end);
    return s;
}

// Zigzag passes along x at height z, from y0 to y1, with a tool axis
void Raster(
    int tool,
    float x0,
    float x1,
    float y0,
    float y1,
    float stepover,
    float z,
    std::vector<CutSegment>& out,
    const float axis[3] = Up
)
{
    int row = 0;
    for (float y = y0; y <= y1 + 1e-4f; y += stepover, row++) {
        const float a[3] = {row % 2 ? x1 : x0, y, z}, b[3] = {row % 2 ? x0 : x1, y, z};
        if (!out.empty()) {
            out.push_back(Line(tool, out.back().end, a));
        }
        out.push_back(Line(tool, a, b));
    }
    for (CutSegment& s : out) {
        std::copy(axis, axis + 3, s.axis0);
        std::copy(axis, axis + 3, s.axis1);
    }
}

void SimulateAll(DexelSimulation& sim)
{
    sim.SetTarget(sim.TotalSteps());
    while (sim.Pending()) {
        sim.Update(1000);
    }
}

}  // namespace

TEST(DexelSimulation, groovesBetweenPassesGetFinerCells)
{
    // parallel passes of a ball end mill at a stepover of a tenth of its diameter, with a
    // vertical and a tilted tool axis (5 axis): the cells resolve the grooves between them. Flat
    // end mills leave no grooves on a floor.
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(20, 10, xyz, triangles);
    const float len = std::sqrt(0.3f * 0.3f + 0.4f * 0.4f + 1);
    const float tilted[3] = {0.3f / len, -0.4f / len, 1 / len};

    struct Case
    {
        bool ball;
        const float* axis;
    };
    for (const Case c : {Case {true, Up}, Case {true, tilted}, Case {false, Up}}) {
        // the ball touches the floor at -1: its center 1 above, the tip a radius along the axis
        // below the center
        std::vector<CutSegment> segments;
        const float tipZ = c.ball ? -1 + 1 - c.axis[2] : -1;
        Raster(0, 6, 14, 7, 13, 0.2f, tipZ, segments, c.axis);
        if (c.ball) {
            for (CutSegment& s : segments) {
                for (float* p : {s.start, s.end}) {
                    p[0] -= c.axis[0];
                    p[1] -= c.axis[1];
                }
            }
        }
        std::vector<std::unique_ptr<CutterProfile>> tools;
        tools.push_back(std::make_unique<CutterProfile>(c.ball ? BallProfile(1, 10) : FlatProfile(1)));
        DexelSimulation sim;
        sim.Start(std::move(tools), segments, xyz, triangles, 10);
        const float fine = sim.Stock().Layout().FineCell();
        if (!c.ball) {
            // a tenth of the tool radius
            EXPECT_NEAR(fine, 0.1f, 0.01f);
            continue;
        }
        // about a quarter of the stepover, finer than for the tool alone (0.087)
        EXPECT_LT(fine, 0.06f) << "tilted " << (c.axis != Up);
        EXPECT_GT(fine, 0.03f);
        SimulateAll(sim);
        const DexelMesh mesh = MeshUnits(sim.Stock());
        double meshSum = 0, exactSum = 0;
        for (uint32_t i : mesh.cutIndices) {
            const DexelVertex& v = mesh.vertices[i];
            if (v.x < 8 || v.x > 12 || v.y < 8.5f || v.y > 11.5f || v.nz < 60) {
                continue;
            }
            const float center = 7 + 0.2f * std::round((v.y - 7) / 0.2f);
            meshSum += std::abs(v.ny / 127.0);
            exactSum += std::abs(center - v.y);
        }
        EXPECT_GT(meshSum, 0.7 * exactSum) << "tilted " << (c.axis != Up);
    }
}

TEST(DexelSimulation, finishingUnderRemovedMaterialRefinesLess)
{
    // a small tool finishing the floor of a pocket roughed by a large tool: its cells are only
    // refined down from the material left by the roughing, not from the top of the stock
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    MakeBox(20, 10, xyz, triangles);
    std::vector<CutSegment> roughing, finishing;
    for (float z = -1; z >= -8; z -= 1) {
        Raster(0, 4, 16, 4, 16, 2.5f, z, roughing);
    }
    Raster(1, 4, 16, 4, 16, 1.0f, -8, finishing);
    auto makeTools = [] {
        std::vector<std::unique_ptr<CutterProfile>> tools;
        tools.push_back(std::make_unique<CutterProfile>(FlatProfile(3)));
        tools.push_back(std::make_unique<CutterProfile>(BallProfile(1, 10)));
        return tools;
    };
    std::vector<CutSegment> both = roughing;
    both.insert(both.end(), finishing.begin(), finishing.end());
    DexelSimulation roughed, solid;
    roughed.Start(makeTools(), both, xyz, triangles, 10);
    solid.Start(makeTools(), finishing, xyz, triangles, 10);
    // volume of the finest leaves, for the small tool
    auto finest = [](const DexelLayout& layout) {
        double volume = 0;
        for (int leaf = 0; leaf < layout.LeafCount(); leaf++) {
            if (layout.Leaf(leaf).level == layout.MaxLevel()) {
                volume += std::pow(BrickCells * layout.CellSize(layout.MaxLevel()), 3.0);
            }
        }
        return volume;
    };
    EXPECT_LT(finest(roughed.Stock().Layout()), finest(solid.Stock().Layout()) / 2);
}



// NOLINTEND(cppcoreguidelines-*,readability-*)
