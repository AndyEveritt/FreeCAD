// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 The FreeCAD project association AISBL              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include "DexelMesher.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

namespace
{

// Largest height difference between neighboring cell tops on one smooth surface, in cells
constexpr float SmoothStep = 3.0f;

class Mesher
{
public:
    Mesher(const DexelStock& stock, const DexelStock& initial, DexelMesh& mesh)
        : mStock(stock)
        , mInitial(initial)
        , mMesh(mesh)
        , mS(stock.CellSize())
        , mMaxStep(SmoothStep * stock.CellSize())
    {}

    void Build(int ix0, int iy0, int ix1, int iy1);

private:
    struct FlatFace
    {
        int x;
        float z;
        bool cut;
    };
    struct FlatRect
    {
        int x0, x1, y0;
        float z;
        bool cut;
        bool used;
    };
    struct SideRun
    {
        int start, end;
        float lo, hi;
        bool cut;
        float n[2];
        bool used;
    };

    float X(int gx) const
    {
        return mStock.X0() + gx * mS;
    }
    float Y(int gy) const
    {
        return mStock.Y0() + gy * mS;
    }

    float CornerZ(int gx, int gy, int cellX, int cellY, float top) const;
    bool TopCorners(int ix, int iy, float hi, float z[4], float n[3]) const;
    void WallNormal(int ix, int iy, int dx, int dy, float z, float n[2]) const;
    void Triangle(const float* a, const float* b, const float* c, const float n[3], bool cut);
    void Quad(const float p[4][3], const float n[3], bool cut);
    uint32_t SharedVertex(int gx, int gy, float z, bool create);
    void SmoothQuad(int ix, int iy, const float z[4], const float n[3], bool cut);
    void AddMarginNormals(int ix0, int iy0, int ix1, int iy1);

    void FlatRow(std::vector<FlatRect>& open, std::vector<FlatFace>& faces, int iy, bool up);
    void EmitFlat(const FlatRect& r, int y1, bool up);
    void Sides(int ix0, int iy0, int ix1, int iy1);

    const DexelStock& mStock;
    const DexelStock& mInitial;
    DexelMesh& mMesh;
    const float mS;
    const float mMaxStep;

    // corner heights by grid corner (chunk with a margin of 1) and surface top, see CornerZ
    struct CornerEntry
    {
        float top;
        float z;
    };
    mutable std::vector<std::vector<CornerEntry>> mCornerCache;
    int mCacheX0 = 0, mCacheY0 = 0, mCacheW = 0, mCacheH = 0;

    // shared vertices of smooth surfaces at the grid corners of the chunk, by height
    int mGX0 = 0, mGY0 = 0, mGX1 = 0, mGY1 = 0;
    std::vector<std::vector<std::pair<float, uint32_t>>> mCorners;
    std::vector<uint32_t> mShared;
    std::vector<float> mNormalSums;  // 3 per shared vertex
};

bool HasHigh(const Interval* iv, int n, float hi)
{
    for (int i = 0; i < n; i++) {
        if (iv[i].hi == hi) {
            return true;
        }
    }
    return false;
}

bool HasLow(const Interval* iv, int n, float lo)
{
    for (int i = 0; i < n; i++) {
        if (iv[i].lo == lo) {
            return true;
        }
    }
    return false;
}

bool Covers(const Interval* iv, int n, float z)
{
    for (int i = 0; i < n; i++) {
        if (iv[i].lo <= z && z <= iv[i].hi) {
            return true;
        }
    }
    return false;
}

// Height of the surface through the top 'top' of cell (cellX, cellY) at grid corner (gx, gy): the
// average of the tops of the cells around the corner that are connected to it by steps of at most
// mMaxStep. The same for every cell on the surface, so neighboring tops share their corners.
// Only cells with one material interval are on smooth surfaces, with several intervals the
// connection would be ambiguous.
float Mesher::CornerZ(int gx, int gy, int cellX, int cellY, float top) const
{
    // every top of the surface at this corner gives the same height
    std::vector<CornerEntry>* cache = nullptr;
    const int cgx = gx - mCacheX0, cgy = gy - mCacheY0;
    if (cgx >= 0 && cgy >= 0 && cgx < mCacheW && cgy < mCacheH) {
        cache = &mCornerCache[cgy * mCacheW + cgx];
        for (const CornerEntry& entry : *cache) {
            if (entry.top == top) {
                return entry.z;
            }
        }
    }

    const int cx[4] = {gx - 1, gx, gx - 1, gx};
    const int cy[4] = {gy - 1, gy - 1, gy, gy};
    float members[4];
    bool used[4] = {};
    int count = 0;
    for (int k = 0; k < 4; k++) {
        if (cx[k] == cellX && cy[k] == cellY) {
            used[k] = true;
            members[count++] = top;
        }
    }
    const Interval* own;
    bool changed = mStock.Cell(cellX, cellY, own) == 1;
    while (changed) {
        changed = false;
        for (int k = 0; k < 4; k++) {
            if (used[k]) {
                continue;
            }
            const Interval* iv;
            const int n = mStock.Cell(cx[k], cy[k], iv);
            if (n != 1) {
                continue;
            }
            float best = mMaxStep;
            int found = -1;
            for (int i = 0; i < n; i++) {
                for (int m = 0; m < count; m++) {
                    const float d = std::abs(iv[i].hi - members[m]);
                    if (d <= best) {
                        best = d;
                        found = i;
                    }
                }
            }
            if (found >= 0) {
                used[k] = true;
                members[count++] = iv[found].hi;
                changed = true;
            }
        }
    }
    float lo = members[0], hi = members[0], sum = 0;
    for (int m = 0; m < count; m++) {
        lo = std::min(lo, members[m]);
        hi = std::max(hi, members[m]);
        sum += members[m];
    }
    const float z = lo == hi ? lo : sum / count;
    if (cache) {
        for (int m = 0; m < count; m++) {
            cache->push_back({members[m], z});
        }
    }
    return z;
}

// Corner heights of the top face of a cell at height hi, and its normal. Returns true if the face
// is flat at height hi.
bool Mesher::TopCorners(int ix, int iy, float hi, float z[4], float n[3]) const
{
    z[0] = CornerZ(ix, iy, ix, iy, hi);
    z[1] = CornerZ(ix + 1, iy, ix, iy, hi);
    z[2] = CornerZ(ix + 1, iy + 1, ix, iy, hi);
    z[3] = CornerZ(ix, iy + 1, ix, iy, hi);
    if (z[0] == hi && z[1] == hi && z[2] == hi && z[3] == hi) {
        n[0] = n[1] = 0;
        n[2] = 1;
        return true;
    }
    const float dx = ((z[1] + z[2]) - (z[0] + z[3])) / (2 * mS);
    const float dy = ((z[3] + z[2]) - (z[0] + z[1])) / (2 * mS);
    const float len = std::sqrt(dx * dx + dy * dy + 1);
    n[0] = -dx / len;
    n[1] = -dy / len;
    n[2] = 1 / len;
    return false;
}

// Horizontal normal of a wall of cell (ix, iy) towards its neighbor (ix + dx, iy + dy) at height
// z, from the outline of the material around the cell, so curved walls are shaded smoothly
void Mesher::WallNormal(int ix, int iy, int dx, int dy, float z, float n[2]) const
{
    auto occupied = [&](int x, int y) {
        const Interval* iv;
        const int count = mStock.Cell(x, y, iv);
        return Covers(iv, count, z) ? 1.0f : 0.0f;
    };
    // 5 x 5 Sobel operator: smoothing across, derivative along the gradient
    const float smooth[5] = {1, 4, 6, 4, 1};
    const float derive[5] = {-1, -2, 0, 2, 1};
    float gx = 0, gy = 0;
    for (int j = -2; j <= 2; j++) {
        for (int i = -2; i <= 2; i++) {
            if (i == 0 && j == 0) {
                continue;
            }
            const float o = occupied(ix + i, iy + j);
            gx += derive[i + 2] * smooth[j + 2] * o;
            gy += smooth[i + 2] * derive[j + 2] * o;
        }
    }
    const float len = std::hypot(gx, gy);
    if (len > 0 && (-gx * dx - gy * dy) / len > 0.2f) {
        n[0] = -gx / len;
        n[1] = -gy / len;
    }
    else {
        n[0] = (float)dx;
        n[1] = (float)dy;
    }
}

uint32_t Mesher::SharedVertex(int gx, int gy, float z, bool create)
{
    if (gx < mGX0 || gx > mGX1 || gy < mGY0 || gy > mGY1) {
        return UINT32_MAX;
    }
    auto& list = mCorners[(gy - mGY0) * (mGX1 - mGX0 + 1) + (gx - mGX0)];
    for (const auto& entry : list) {
        if (entry.first == z) {
            return entry.second;
        }
    }
    if (!create) {
        return UINT32_MAX;
    }
    const auto index = (uint32_t)mMesh.vertices.size();
    mMesh.vertices.push_back({X(gx), Y(gy), z, 0, 0, 0, 0});
    list.emplace_back(z, index);
    mShared.push_back(index);
    mNormalSums.insert(mNormalSums.end(), {0, 0, 0});
    return index;
}

// Top face of a cell on a smooth surface, with shared vertices
void Mesher::SmoothQuad(int ix, int iy, const float z[4], const float n[3], bool cut)
{
    const int gx[4] = {ix, ix + 1, ix + 1, ix};
    const int gy[4] = {iy, iy, iy + 1, iy + 1};
    uint32_t v[4];
    for (int k = 0; k < 4; k++) {
        v[k] = SharedVertex(gx[k], gy[k], z[k], true);
        const size_t sum = 3
            * (std::lower_bound(mShared.begin(), mShared.end(), v[k]) - mShared.begin());
        for (int c = 0; c < 3; c++) {
            mNormalSums[sum + c] += n[c];
        }
    }
    // the corners are counter clockwise seen from above, the face normal points up
    auto& indices = cut ? mMesh.cutIndices : mMesh.stockIndices;
    indices.insert(indices.end(), {v[0], v[1], v[2], v[0], v[2], v[3]});
}

// Add the normals of the smooth faces of the cells around the chunk to the shared vertices on the
// chunk border, so the shading continues across chunks
void Mesher::AddMarginNormals(int ix0, int iy0, int ix1, int iy1)
{
    for (int iy = iy0 - 1; iy <= iy1; iy++) {
        for (int ix = ix0 - 1; ix <= ix1; ix++) {
            if (ix >= ix0 && ix < ix1 && iy >= iy0 && iy < iy1) {
                continue;
            }
            const Interval* iv;
            const int n = mStock.Cell(ix, iy, iv);
            for (int i = 0; i < n; i++) {
                float z[4], normal[3];
                if (TopCorners(ix, iy, iv[i].hi, z, normal)) {
                    continue;
                }
                const int gx[4] = {ix, ix + 1, ix + 1, ix};
                const int gy[4] = {iy, iy, iy + 1, iy + 1};
                for (int k = 0; k < 4; k++) {
                    const uint32_t v = SharedVertex(gx[k], gy[k], z[k], false);
                    if (v == UINT32_MAX) {
                        continue;
                    }
                    const size_t sum = 3
                        * (std::lower_bound(mShared.begin(), mShared.end(), v) - mShared.begin());
                    for (int c = 0; c < 3; c++) {
                        mNormalSums[sum + c] += normal[c];
                    }
                }
            }
        }
    }
}

void Mesher::Triangle(const float* a, const float* b, const float* c, const float n[3], bool cut)
{
    // counter clockwise around the normal
    const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const float e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    const float cross[3] = {
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    };
    const float dot = cross[0] * n[0] + cross[1] * n[1] + cross[2] * n[2];
    if (dot == 0) {
        return;  // degenerate
    }
    if (dot < 0) {
        std::swap(b, c);
    }
    const auto base = (uint32_t)mMesh.vertices.size();
    for (const float* v : {a, b, c}) {
        mMesh.vertices.push_back({
            v[0],
            v[1],
            v[2],
            (int8_t)std::lround(n[0] * 127),
            (int8_t)std::lround(n[1] * 127),
            (int8_t)std::lround(n[2] * 127),
            0,
        });
    }
    auto& indices = cut ? mMesh.cutIndices : mMesh.stockIndices;
    indices.insert(indices.end(), {base, base + 1, base + 2});
}

void Mesher::Quad(const float p[4][3], const float n[3], bool cut)
{
    Triangle(p[0], p[1], p[2], n, cut);
    Triangle(p[0], p[2], p[3], n, cut);
}

void Mesher::EmitFlat(const FlatRect& r, int y1, bool up)
{
    const float xa = X(r.x0), xb = X(r.x1 + 1), ya = Y(r.y0), yb = Y(y1 + 1);
    const float p[4][3] = {{xa, ya, r.z}, {xb, ya, r.z}, {xb, yb, r.z}, {xa, yb, r.z}};
    const float n[3] = {0, 0, up ? 1.0f : -1.0f};
    Quad(p, n, r.cut);
}

// Merge the flat faces of a row into runs, and the runs into rectangles over rows
void Mesher::FlatRow(std::vector<FlatRect>& open, std::vector<FlatFace>& faces, int iy, bool up)
{
    std::sort(faces.begin(), faces.end(), [](const FlatFace& a, const FlatFace& b) {
        if (a.z != b.z) {
            return a.z < b.z;
        }
        if (a.cut != b.cut) {
            return a.cut < b.cut;
        }
        return a.x < b.x;
    });
    for (auto& rect : open) {
        rect.used = false;
    }
    for (size_t i = 0; i < faces.size();) {
        size_t j = i + 1;
        while (j < faces.size() && faces[j].z == faces[i].z && faces[j].cut == faces[i].cut
               && faces[j].x == faces[j - 1].x + 1) {
            j++;
        }
        const int x0 = faces[i].x, x1 = faces[j - 1].x;
        bool extended = false;
        for (auto& rect : open) {
            if (!rect.used && rect.x0 == x0 && rect.x1 == x1 && rect.z == faces[i].z
                && rect.cut == faces[i].cut) {
                rect.used = true;
                extended = true;
                break;
            }
        }
        if (!extended) {
            open.push_back({x0, x1, iy, faces[i].z, faces[i].cut, true});
        }
        i = j;
    }
    for (size_t i = 0; i < open.size();) {
        if (!open[i].used) {
            EmitFlat(open[i], iy - 1, up);
            open[i] = open.back();
            open.pop_back();
        }
        else {
            i++;
        }
    }
}

void Mesher::Build(int ix0, int iy0, int ix1, int iy1)
{
    mGX0 = ix0;
    mGY0 = iy0;
    mGX1 = ix1;
    mGY1 = iy1;
    mCorners.assign((size_t)(ix1 - ix0 + 1) * (iy1 - iy0 + 1), {});
    mCacheX0 = ix0 - 1;
    mCacheY0 = iy0 - 1;
    mCacheW = ix1 - ix0 + 3;
    mCacheH = iy1 - iy0 + 3;
    mCornerCache.assign((size_t)mCacheW * mCacheH, {});

    // tops and bottoms
    std::vector<FlatRect> openTops, openBottoms;
    std::vector<FlatFace> tops, bottoms;
    for (int iy = iy0; iy < iy1; iy++) {
        tops.clear();
        bottoms.clear();
        for (int ix = ix0; ix < ix1; ix++) {
            const Interval* iv;
            const int n = mStock.Cell(ix, iy, iv);
            if (n == 0) {
                continue;
            }
            const Interval* iv0;
            const int n0 = mInitial.Cell(ix, iy, iv0);
            for (int i = 0; i < n; i++) {
                const float hi = iv[i].hi;
                const bool topCut = !HasHigh(iv0, n0, hi);
                float z[4], normal[3];
                if (TopCorners(ix, iy, hi, z, normal)) {
                    tops.push_back({ix, hi, topCut});
                }
                else {
                    SmoothQuad(ix, iy, z, normal, topCut);
                }
                bottoms.push_back({ix, iv[i].lo, !HasLow(iv0, n0, iv[i].lo)});
            }
        }
        FlatRow(openTops, tops, iy, true);
        FlatRow(openBottoms, bottoms, iy, false);
    }
    for (auto& rect : openTops) {
        EmitFlat(rect, iy1 - 1, true);
    }
    for (auto& rect : openBottoms) {
        EmitFlat(rect, iy1 - 1, false);
    }

    // smooth vertex normals
    AddMarginNormals(ix0, iy0, ix1, iy1);
    for (size_t k = 0; k < mShared.size(); k++) {
        const float* sum = &mNormalSums[3 * k];
        const float len = std::sqrt(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
        DexelVertex& v = mMesh.vertices[mShared[k]];
        v.nx = (int8_t)std::lround(sum[0] / len * 127);
        v.ny = (int8_t)std::lround(sum[1] / len * 127);
        v.nz = (int8_t)std::lround(sum[2] / len * 127);
    }

    Sides(ix0, iy0, ix1, iy1);
}

// Side faces of every cell towards its 4 neighbors, where the neighbor has no material and the
// tops are not on one smooth surface. Rectangular faces are merged along their line.
void Mesher::Sides(int ix0, int iy0, int ix1, int iy1)
{
    Interval parts[130];
    std::vector<SideRun> open;
    for (int dir = 0; dir < 4; dir++) {
        const int dx = dir == 0 ? 1 : dir == 1 ? -1 : 0;
        const int dy = dir == 2 ? 1 : dir == 3 ? -1 : 0;
        const bool alongY = dx != 0;  // faces of a line of cells with constant x
        const int lines0 = alongY ? ix0 : iy0, lines1 = alongY ? ix1 : iy1;
        const int along0 = alongY ? iy0 : ix0, along1 = alongY ? iy1 : ix1;
        for (int line = lines0; line < lines1; line++) {
            // grid line of the faces
            const int g = line + (dx > 0 || dy > 0 ? 1 : 0);
            const float fixed = alongY ? X(g) : Y(g);
            auto corner = [&](int a, float z, float p[3]) {
                p[0] = alongY ? fixed : X(a);
                p[1] = alongY ? Y(a) : fixed;
                p[2] = z;
            };
            auto emitRun = [&](const SideRun& r) {
                float p[4][3];
                corner(r.start, r.lo, p[0]);
                corner(r.end, r.lo, p[1]);
                corner(r.end, r.hi, p[2]);
                corner(r.start, r.hi, p[3]);
                const float normal[3] = {r.n[0], r.n[1], 0};
                Quad(p, normal, r.cut);
            };
            open.clear();
            for (int along = along0; along < along1; along++) {
                for (auto& run : open) {
                    run.used = false;
                }
                const int ix = alongY ? line : along;
                const int iy = alongY ? along : line;
                const int nx = ix + dx, ny = iy + dy;
                const Interval* iv;
                const int n = mStock.Cell(ix, iy, iv);
                const Interval* nb;
                const int nn = mStock.Cell(nx, ny, nb);
                const Interval* nb0;
                const int nn0 = mInitial.Cell(nx, ny, nb0);
                for (int i = 0; i < n; i++) {
                    // parts of the interval not covered by the neighbor
                    int m = 0;
                    float lo = iv[i].lo;
                    for (int k = 0; k < nn && lo < iv[i].hi && m < 128; k++) {
                        if (nb[k].hi <= lo) {
                            continue;
                        }
                        if (nb[k].lo >= iv[i].hi) {
                            break;
                        }
                        if (nb[k].lo > lo) {
                            parts[m++] = {lo, nb[k].lo};
                        }
                        lo = std::max(lo, nb[k].hi);
                    }
                    if (lo < iv[i].hi) {
                        parts[m++] = {lo, iv[i].hi};
                    }
                    for (int k = 0; k < m; k++) {
                        const float a = parts[k].lo, b = parts[k].hi;
                        const bool topFollows = b == iv[i].hi;
                        const bool bottomFollows = HasHigh(nb, nn, a);
                        if (topFollows && bottomFollows && b - a <= mMaxStep && n == 1 && nn == 1) {
                            continue;  // one smooth surface
                        }
                        const bool cut = Covers(nb0, nn0, (a + b) / 2);
                        float wall[2];
                        WallNormal(ix, iy, dx, dy, (a + b) / 2, wall);
                        // the face ends at the grid corners 'along' and 'along + 1'
                        const int c0 = along, c1 = along + 1;
                        const int g0x = alongY ? g : c0, g0y = alongY ? c0 : g;
                        const int g1x = alongY ? g : c1, g1y = alongY ? c1 : g;
                        float top0 = b, top1 = b, bot0 = a, bot1 = a;
                        if (topFollows) {
                            top0 = CornerZ(g0x, g0y, ix, iy, b);
                            top1 = CornerZ(g1x, g1y, ix, iy, b);
                        }
                        if (bottomFollows) {
                            bot0 = CornerZ(g0x, g0y, nx, ny, a);
                            bot1 = CornerZ(g1x, g1y, nx, ny, a);
                        }
                        if (top0 == b && top1 == b && bot0 == a && bot1 == a) {
                            // rectangle, extend a run
                            bool extended = false;
                            for (auto& run : open) {
                                if (!run.used && run.end == along && run.lo == a && run.hi == b
                                    && run.cut == cut && run.n[0] == wall[0] && run.n[1] == wall[1]) {
                                    run.end = along + 1;
                                    run.used = true;
                                    extended = true;
                                    break;
                                }
                            }
                            if (!extended) {
                                open.push_back({along, along + 1, a, b, cut, {wall[0], wall[1]}, true});
                            }
                            continue;
                        }
                        float p[4][3];
                        corner(c0, bot0, p[0]);
                        corner(c1, bot1, p[1]);
                        corner(c1, top1, p[2]);
                        corner(c0, top0, p[3]);
                        const float normal[3] = {wall[0], wall[1], 0};
                        Quad(p, normal, cut);
                    }
                }
                for (size_t r = 0; r < open.size();) {
                    if (!open[r].used) {
                        emitRun(open[r]);
                        open[r] = open.back();
                        open.pop_back();
                    }
                    else {
                        r++;
                    }
                }
            }
            for (auto& run : open) {
                emitRun(run);
            }
        }
    }
}

}  // namespace

void BuildDexelMesh(
    const DexelStock& stock,
    const DexelStock& initial,
    int ix0,
    int iy0,
    int ix1,
    int iy1,
    DexelMesh& mesh
)
{
    mesh.vertices.clear();
    mesh.stockIndices.clear();
    mesh.cutIndices.clear();
    Mesher(stock, initial, mesh).Build(ix0, iy0, ix1, iy1);
}

}  // namespace CAMSimulator
