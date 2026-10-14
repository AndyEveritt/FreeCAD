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

#include "DexelStock.h"
#include "ThreadPool.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

namespace
{

constexpr float MinThickness = 1e-4f;

int GetIntervals(const DexelTile& tile, int c, const Interval*& out)
{
    const int n = tile.count[c];
    if (n <= 2) {
        out = tile.inlineIntervals[c];
        return n;
    }
    out = tile.large[c].data();
    return (int)tile.large[c].size();
}

void SetIntervals(DexelTile& tile, int c, const Interval* intervals, int n)
{
    if (n <= 2) {
        if (tile.count[c] > 2) {
            tile.large[c].clear();
        }
        tile.count[c] = (uint8_t)n;
        std::copy(intervals, intervals + n, tile.inlineIntervals[c]);
        return;
    }
    if (tile.large.empty()) {
        tile.large.resize(DexelTileCells);
    }
    tile.large[c].assign(intervals, intervals + n);
    tile.count[c] = 3;
}

// a - r for sorted, disjoint interval lists. Returns the number of intervals written to out,
// which must have room for na + nr intervals.
int Subtract(const Interval* a, int na, const Interval* r, int nr, Interval* out)
{
    int n = 0;
    int j = 0;
    for (int i = 0; i < na; i++) {
        float lo = a[i].lo;
        const float hi = a[i].hi;
        while (j < nr && r[j].hi <= lo) {
            j++;
        }
        for (int k = j; k < nr && r[k].lo < hi; k++) {
            if (r[k].lo > lo + MinThickness) {
                out[n++] = {lo, r[k].lo};
            }
            lo = std::max(lo, r[k].hi);
            if (lo >= hi) {
                break;
            }
        }
        if (hi - lo > MinThickness) {
            out[n++] = {lo, hi};
        }
    }
    return n;
}

float DistanceToSegment(float px, float py, float ax, float ay, float bx, float by)
{
    const float dx = bx - ax, dy = by - ay;
    const float len2 = dx * dx + dy * dy;
    float t = 0;
    if (len2 > 0) {
        t = std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0f, 1.0f);
    }
    return std::hypot(px - ax - t * dx, py - ay - t * dy);
}

bool IsVertical(const float a[3])
{
    return std::abs(a[0]) < 1e-6f && std::abs(a[1]) < 1e-6f && a[2] > 0;
}

}  // namespace

void DexelStock::Init(float x0, float y0, float cell, int nx, int ny)
{
    mX0 = x0;
    mY0 = y0;
    mCell = cell;
    mNX = nx;
    mNY = ny;
    mTilesX = (nx + DexelTileSize - 1) / DexelTileSize;
    mTilesY = (ny + DexelTileSize - 1) / DexelTileSize;
    mTiles.assign((size_t)mTilesX * mTilesY, nullptr);
    mDirty.assign(mTiles.size(), 1);
}

void DexelStock::SetMesh(
    const std::vector<float>& xyz,
    const std::vector<uint32_t>& triangles,
    ThreadPool& pool
)
{
    // triangles overlapping each tile
    std::vector<std::vector<int>> tileTriangles(mTiles.size());
    const int nTriangles = (int)triangles.size() / 3;
    for (int t = 0; t < nTriangles; t++) {
        float minx = 1e30f, maxx = -1e30f, miny = 1e30f, maxy = -1e30f;
        for (int k = 0; k < 3; k++) {
            const float* p = &xyz[3 * triangles[3 * t + k]];
            minx = std::min(minx, p[0]);
            maxx = std::max(maxx, p[0]);
            miny = std::min(miny, p[1]);
            maxy = std::max(maxy, p[1]);
        }
        const int ix0 = std::max(0, (int)std::floor((minx - mX0) / mCell - 0.5f));
        const int ix1 = std::min(mNX - 1, (int)std::ceil((maxx - mX0) / mCell - 0.5f));
        const int iy0 = std::max(0, (int)std::floor((miny - mY0) / mCell - 0.5f));
        const int iy1 = std::min(mNY - 1, (int)std::ceil((maxy - mY0) / mCell - 0.5f));
        if (ix0 > ix1 || iy0 > iy1) {
            continue;
        }
        for (int ty = iy0 >> DexelTileBits; ty <= iy1 >> DexelTileBits; ty++) {
            for (int tx = ix0 >> DexelTileBits; tx <= ix1 >> DexelTileBits; tx++) {
                tileTriangles[ty * mTilesX + tx].push_back(t);
            }
        }
    }

    const int workers = pool.Size();
    pool.Run([&](int worker) {
        struct Hit
        {
            double z;
            int winding;
        };
        std::vector<std::vector<Hit>> hits(DexelTileCells);
        std::vector<Interval> intervals;
        for (int tile = worker; tile < (int)mTiles.size(); tile += workers) {
            const auto& list = tileTriangles[tile];
            if (list.empty()) {
                mTiles[tile] = nullptr;
                continue;
            }
            for (auto& h : hits) {
                h.clear();
            }
            const int tx0 = (tile % mTilesX) * DexelTileSize;
            const int ty0 = (tile / mTilesX) * DexelTileSize;
            for (int t : list) {
                const float* p[3];
                for (int k = 0; k < 3; k++) {
                    p[k] = &xyz[3 * triangles[3 * t + k]];
                }
                double area = (double)(p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (double)(p[1][1] - p[0][1]) * (p[2][0] - p[0][0]);
                if (std::abs(area) < 1e-12) {
                    continue;  // vertical
                }
                // going up, a downward facing triangle enters the material
                const int winding = area < 0 ? 1 : -1;
                if (area < 0) {
                    std::swap(p[1], p[2]);
                    area = -area;
                }
                for (int ly = 0; ly < DexelTileSize; ly++) {
                    const int iy = ty0 + ly;
                    if (iy >= mNY) {
                        break;
                    }
                    const double cy = mY0 + (iy + 0.5) * mCell;
                    for (int lx = 0; lx < DexelTileSize; lx++) {
                        const int ix = tx0 + lx;
                        if (ix >= mNX) {
                            break;
                        }
                        const double cx = mX0 + (ix + 0.5) * mCell;
                        double w[3];
                        bool inside = true;
                        for (int k = 0; k < 3 && inside; k++) {
                            const float* a = p[(k + 1) % 3];
                            const float* b = p[(k + 2) % 3];
                            // relative to the cell center, so the reversed edge of the
                            // neighboring triangle gives exactly the negated value
                            const double ax = a[0] - cx, ay = a[1] - cy;
                            const double bx = b[0] - cx, by = b[1] - cy;
                            w[k] = ax * by - ay * bx;
                            // top left rule, so points on shared edges belong to one triangle
                            const bool topLeft = (a[1] == b[1] && b[0] < a[0]) || b[1] < a[1];
                            inside = w[k] > 0 || (w[k] == 0 && topLeft);
                        }
                        if (!inside) {
                            continue;
                        }
                        // relative to a corner, so flat triangles give exactly their height
                        const double z = p[0][2]
                            + (w[1] * (p[1][2] - p[0][2]) + w[2] * (p[2][2] - p[0][2])) / area;
                        hits[ly * DexelTileSize + lx].push_back({z, winding});
                    }
                }
            }

            auto newTile = std::make_shared<DexelTile>();
            bool any = false;
            for (int c = 0; c < DexelTileCells; c++) {
                auto& h = hits[c];
                if (h.empty()) {
                    continue;
                }
                std::sort(h.begin(), h.end(), [](const Hit& a, const Hit& b) { return a.z < b.z; });
                intervals.clear();
                int w = 0;
                double start = 0;
                for (const Hit& hit : h) {
                    const int before = w;
                    w += hit.winding;
                    if (before <= 0 && w > 0) {
                        start = hit.z;
                    }
                    else if (before > 0 && w <= 0) {
                        if (!intervals.empty() && start - intervals.back().hi < MinThickness) {
                            intervals.back().hi = (float)hit.z;
                        }
                        else if (hit.z - start > MinThickness) {
                            intervals.push_back({(float)start, (float)hit.z});
                        }
                    }
                }
                if (!intervals.empty()) {
                    SetIntervals(*newTile, c, intervals.data(), (int)intervals.size());
                    any = true;
                }
            }
            mTiles[tile] = any ? newTile : nullptr;
            mDirty[tile] = 1;
        }
    });
}

int DexelStock::Cell(int ix, int iy, const Interval*& out) const
{
    if (ix < 0 || iy < 0 || ix >= mNX || iy >= mNY) {
        return 0;
    }
    const DexelTile* tile = mTiles[TileIndex(ix, iy)].get();
    if (!tile) {
        return 0;
    }
    return GetIntervals(*tile, LocalIndex(ix, iy), out);
}

void DexelStock::Freeze()
{
    // frozen tiles may be read by other threads, only tiles owned by this stock are written
    for (auto& tile : mTiles) {
        if (tile && !tile->frozen) {
            tile->frozen = true;
        }
    }
}

void DexelStock::Restore(const Snapshot& snapshot)
{
    for (size_t i = 0; i < mTiles.size(); i++) {
        if (mTiles[i] != snapshot[i]) {
            mDirty[i] = 1;
        }
    }
    mTiles = snapshot;
}

std::vector<int> DexelStock::TakeDirtyTiles()
{
    std::vector<int> dirty;
    for (size_t i = 0; i < mDirty.size(); i++) {
        if (mDirty[i]) {
            dirty.push_back((int)i);
            mDirty[i] = 0;
        }
    }
    return dirty;
}

long DexelStock::TakeNewTileBytes()
{
    return mNewTileBytes.exchange(0);
}

long DexelStock::EstimateWork(const CutMove& move) const
{
    const float r = move.tool->Radius() + mCell;
    const float dx = move.p1[0] - move.p0[0];
    const float dy = move.p1[1] - move.p0[1];
    const float length = std::hypot(dx, dy);
    float area = (length + 2 * r) * 2 * r;
    if (!IsVertical(move.a0) || !IsVertical(move.a1)) {
        // sampled instances, each covering the tool footprint
        const float h = move.tool->Height();
        float dist = std::hypot(length, move.p1[2] - move.p0[2]);
        for (int k = 0; k < 3; k++) {
            dist = std::max(dist, std::abs(move.a1[k] - move.a0[k]) * h);
        }
        const float instances = 1 + dist / (mCell * 0.5f);
        area = instances * (2 * r + 2 * h) * (2 * r + 2 * h);
    }
    else {
        const float dz = std::abs(move.p1[2] - move.p0[2]);
        area *= std::max(1.0f, dz / (mCell * 0.25f));
    }
    return (long)(area / (mCell * mCell)) + 1;
}

long DexelStock::Cut(const std::vector<CutMove>& moves, ThreadPool& pool, const std::atomic<bool>* cancel)
{
    if (moves.empty()) {
        return 0;
    }
    const int workers = pool.Size();
    std::vector<long> work(workers, 0);
    pool.Run([&](int worker) {
        for (const CutMove& move : moves) {
            if (cancel && *cancel) {
                return;
            }
            CutWorker(move, worker, workers, work[worker]);
        }
    });
    long total = 0;
    for (long w : work) {
        total += w;
    }
    return total;
}

void DexelStock::CutWorker(const CutMove& move, int worker, int workers, long& work)
{
    const CutterProfile& tool = *move.tool;
    if (IsVertical(move.a0) && IsVertical(move.a1) && tool.IsRadiallyMonotonic()) {
        // Split moves with a height change into pieces that change the height by at most a
        // quarter cell, each removes the material of its closest approach over its height range.
        const float dz = std::abs(move.p1[2] - move.p0[2]);
        const float dxy = std::hypot(move.p1[0] - move.p0[0], move.p1[1] - move.p0[1]);
        const float tolerance = mCell * 0.25f;
        const int pieces = (dxy > 1e-6f && dz > tolerance) ? (int)std::ceil(dz / tolerance) : 1;
        for (int i = 0; i < pieces; i++) {
            float a[3], b[3];
            for (int k = 0; k < 3; k++) {
                a[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * i / pieces;
                b[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * (i + 1) / pieces;
            }
            CutVerticalSegment(tool, a, b, worker, workers, work);
        }
        return;
    }

    // General motion: tool instances, close enough that neither the tip nor the top of the tool
    // moves more than half a cell between them.
    const float h = tool.Height();
    float dist = 0;
    for (int k = 0; k < 3; k++) {
        const float tip = move.p1[k] - move.p0[k];
        const float top = tip + (move.a1[k] - move.a0[k]) * h;
        dist = std::max({dist, std::abs(tip), std::abs(top)});
    }
    const int steps = std::max(1, (int)std::ceil(dist / (mCell * 0.5f)));
    std::vector<Interval> scratch;
    for (int i = 0; i <= steps; i++) {
        const float t = (float)i / steps;
        float tip[3], axis[3];
        for (int k = 0; k < 3; k++) {
            tip[k] = move.p0[k] + (move.p1[k] - move.p0[k]) * t;
            axis[k] = move.a0[k] + (move.a1[k] - move.a0[k]) * t;
        }
        const float len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        for (float& a : axis) {
            a /= len;
        }
        CutInstance(tool, tip, axis, worker, workers, work, scratch);
    }
}

void DexelStock::CutVerticalSegment(
    const CutterProfile& tool,
    const float a[3],
    const float b[3],
    int worker,
    int workers,
    long& work
)
{
    const float r = tool.Radius();
    const float zmin = std::min(a[2], b[2]);
    const float zmax = std::max(a[2], b[2]);
    const int ix0 = std::max(0, (int)std::floor((std::min(a[0], b[0]) - r - mX0) / mCell));
    const int ix1 = std::min(mNX - 1, (int)std::floor((std::max(a[0], b[0]) + r - mX0) / mCell));
    const int iy0 = std::max(0, (int)std::floor((std::min(a[1], b[1]) - r - mY0) / mCell));
    const int iy1 = std::min(mNY - 1, (int)std::floor((std::max(a[1], b[1]) + r - mY0) / mCell));
    if (ix0 > ix1 || iy0 > iy1) {
        return;
    }
    const float dx = b[0] - a[0], dy = b[1] - a[1];
    const float len2 = dx * dx + dy * dy;
    // With one solid interval per dexel, the removed material is one interval from the lowest
    // bottom to the highest top of the tool along the move. They are taken from the tool at the
    // start, the end and the closest approach of the move. Otherwise the solid of the closest
    // approach is extended over the height range of the move, which can remove slightly more.
    const bool single = tool.HasSingleAxialIntervals() && zmin != zmax;

    Interval removal[64];
    for (int ty = iy0 >> DexelTileBits; ty <= iy1 >> DexelTileBits; ty++) {
        for (int tx = ix0 >> DexelTileBits; tx <= ix1 >> DexelTileBits; tx++) {
            const int tile = ty * mTilesX + tx;
            if (tile % workers != worker || !mTiles[tile]) {
                continue;
            }
            const int cy0 = std::max(iy0, ty << DexelTileBits);
            const int cy1 = std::min(iy1, ((ty + 1) << DexelTileBits) - 1);
            const int cx0 = std::max(ix0, tx << DexelTileBits);
            const int cx1 = std::min(ix1, ((tx + 1) << DexelTileBits) - 1);
            for (int iy = cy0; iy <= cy1; iy++) {
                const float cy = mY0 + (iy + 0.5f) * mCell;
                for (int ix = cx0; ix <= cx1; ix++) {
                    work++;
                    const float cx = mX0 + (ix + 0.5f) * mCell;
                    float t = 0;
                    if (len2 > 0) {
                        t = std::clamp(((cx - a[0]) * dx + (cy - a[1]) * dy) / len2, 0.0f, 1.0f);
                    }
                    const float d = std::hypot(cx - a[0] - t * dx, cy - a[1] - t * dy);
                    if (d > r) {
                        continue;
                    }
                    const Interval* solid;
                    int nr = 0;
                    if (single) {
                        const float candidates[3] = {0, t, 1};
                        float lo = 1e30f, hi = -1e30f;
                        for (float tc : candidates) {
                            const float dc = std::hypot(cx - a[0] - tc * dx, cy - a[1] - tc * dy);
                            if (tool.AxialIntervals(dc, solid) == 1) {
                                const float z = a[2] + tc * (b[2] - a[2]);
                                lo = std::min(lo, solid[0].lo + z);
                                hi = std::max(hi, solid[0].hi + z);
                            }
                        }
                        if (lo < hi) {
                            removal[nr++] = {lo, hi};
                        }
                    }
                    else {
                        const int n = std::min(64, tool.AxialIntervals(d, solid));
                        for (int k = 0; k < n; k++) {
                            const Interval iv {solid[k].lo + zmin, solid[k].hi + zmax};
                            if (nr > 0 && iv.lo <= removal[nr - 1].hi) {
                                removal[nr - 1].hi = std::max(removal[nr - 1].hi, iv.hi);
                            }
                            else {
                                removal[nr++] = iv;
                            }
                        }
                    }
                    if (nr > 0) {
                        SubtractCell(tile, LocalIndex(ix, iy), removal, nr);
                    }
                }
            }
        }
    }
}

void DexelStock::CutInstance(
    const CutterProfile& tool,
    const float tip[3],
    const float axis[3],
    int worker,
    int workers,
    long& work,
    std::vector<Interval>& scratch
)
{
    const float r = tool.Radius();
    const float h = tool.Height();
    const float top[3] = {tip[0] + axis[0] * h, tip[1] + axis[1] * h, tip[2] + axis[2] * h};
    const int ix0 = std::max(0, (int)std::floor((std::min(tip[0], top[0]) - r - mX0) / mCell));
    const int ix1 = std::min(mNX - 1, (int)std::floor((std::max(tip[0], top[0]) + r - mX0) / mCell));
    const int iy0 = std::max(0, (int)std::floor((std::min(tip[1], top[1]) - r - mY0) / mCell));
    const int iy1 = std::min(mNY - 1, (int)std::floor((std::max(tip[1], top[1]) + r - mY0) / mCell));
    if (ix0 > ix1 || iy0 > iy1) {
        return;
    }

    // tool coordinate system
    float e1[3];
    if (std::abs(axis[0]) < 0.9f) {
        e1[0] = 0;
        e1[1] = axis[2];
        e1[2] = -axis[1];
    }
    else {
        e1[0] = -axis[2];
        e1[1] = 0;
        e1[2] = axis[0];
    }
    const float len = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
    for (float& v : e1) {
        v /= len;
    }
    const float e2[3] = {
        axis[1] * e1[2] - axis[2] * e1[1],
        axis[2] * e1[0] - axis[0] * e1[2],
        axis[0] * e1[1] - axis[1] * e1[0],
    };
    // the dexel direction (world z) in tool coordinates
    const float dir[3] = {e1[2], e2[2], axis[2]};
    const bool vertical = IsVertical(axis);

    for (int ty = iy0 >> DexelTileBits; ty <= iy1 >> DexelTileBits; ty++) {
        for (int tx = ix0 >> DexelTileBits; tx <= ix1 >> DexelTileBits; tx++) {
            const int tile = ty * mTilesX + tx;
            if (tile % workers != worker || !mTiles[tile]) {
                continue;
            }
            const int cy0 = std::max(iy0, ty << DexelTileBits);
            const int cy1 = std::min(iy1, ((ty + 1) << DexelTileBits) - 1);
            const int cx0 = std::max(ix0, tx << DexelTileBits);
            const int cx1 = std::min(ix1, ((tx + 1) << DexelTileBits) - 1);
            for (int iy = cy0; iy <= cy1; iy++) {
                const float cy = mY0 + (iy + 0.5f) * mCell;
                for (int ix = cx0; ix <= cx1; ix++) {
                    work++;
                    const float cx = mX0 + (ix + 0.5f) * mCell;
                    // the dexel can only hit the tool near its axis
                    if (DistanceToSegment(cx, cy, tip[0], tip[1], top[0], top[1]) > r) {
                        continue;
                    }
                    if (vertical) {
                        const Interval* solid;
                        const int n = tool.AxialIntervals(std::hypot(cx - tip[0], cy - tip[1]), solid);
                        scratch.clear();
                        for (int k = 0; k < n; k++) {
                            scratch.push_back({solid[k].lo + tip[2], solid[k].hi + tip[2]});
                        }
                    }
                    else {
                        const float o[3] = {cx - tip[0], cy - tip[1], -tip[2]};
                        const float origin[3] = {
                            o[0] * e1[0] + o[1] * e1[1] + o[2] * e1[2],
                            o[0] * e2[0] + o[1] * e2[1] + o[2] * e2[2],
                            o[0] * axis[0] + o[1] * axis[1] + o[2] * axis[2],
                        };
                        tool.LineIntervals(origin, dir, scratch);
                    }
                    if (!scratch.empty()) {
                        SubtractCell(tile, LocalIndex(ix, iy), scratch.data(), (int)scratch.size());
                    }
                }
            }
        }
    }
}

bool DexelStock::SubtractCell(int tileIndex, int c, const Interval* removal, int nr)
{
    DexelTile* tile = mTiles[tileIndex].get();
    const Interval* current;
    const int n = GetIntervals(*tile, c, current);
    if (n == 0 || removal[nr - 1].hi <= current[0].lo || removal[0].lo >= current[n - 1].hi) {
        return false;
    }

    Interval local[66];
    std::vector<Interval> heap;
    Interval* result = local;
    if (n + nr > 66) {
        heap.resize(n + nr);
        result = heap.data();
    }
    const int m = Subtract(current, n, removal, nr, result);
    if (m == n && std::equal(result, result + m, current, [](const Interval& a, const Interval& b) {
            return a.lo == b.lo && a.hi == b.hi;
        })) {
        return false;
    }

    if (tile->frozen) {
        auto copy = std::make_shared<DexelTile>(*tile);
        copy->frozen = false;
        mTiles[tileIndex] = copy;
        tile = copy.get();
        mNewTileBytes += (long)sizeof(DexelTile);
    }
    SetIntervals(*tile, c, result, m);
    mDirty[tileIndex] = 1;
    return true;
}

}  // namespace CAMSimulator
