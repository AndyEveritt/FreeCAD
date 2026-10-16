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

#include "MaterialLayout.h"

#include "ThreadPool.h"

#include <algorithm>
#include <atomic>

namespace CAMSimulator::Material
{

namespace
{

// Bricks of a level are keyed by their coordinates in bricks of that level, 21 bits each
inline uint64_t Key(uint64_t x, uint64_t y, uint64_t z)
{
    return x | (y << 21) | (z << 42);
}
inline void Unkey(uint64_t k, int& x, int& y, int& z)
{
    x = (int)(k & 0x1FFFFF);
    y = (int)((k >> 21) & 0x1FFFFF);
    z = (int)(k >> 42);
}
inline uint64_t ParentKey(uint64_t k)
{
    int x, y, z;
    Unkey(k, x, y, z);
    return Key(x >> 1, y >> 1, z >> 1);
}

// Distance from a box to a segment, a lower bound: from the box center minus the half diagonal
double BoxSegmentDistance(const double c[3], double half, const double a[3], const double b[3])
{
    double d[3], w[3];
    for (int k = 0; k < 3; k++) {
        d[k] = b[k] - a[k];
        w[k] = c[k] - a[k];
    }
    const double dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    const double t = dd > 0 ? std::clamp((w[0] * d[0] + w[1] * d[1] + w[2] * d[2]) / dd, 0.0, 1.0) : 0;
    double dist = 0;
    for (int k = 0; k < 3; k++) {
        const double e = w[k] - t * d[k];
        dist += e * e;
    }
    return std::sqrt(dist) - half;
}

}  // namespace

struct Layout::Builder::Data
{
    double origin[3];
    double unit;
    int levels;
    int roots[3];
    // requested bricks per thread and level
    std::vector<std::vector<std::vector<uint64_t>>> requests;
};

Layout::Builder::Builder(const double lo[3], const double hi[3], double rootCell, int levels, int threads)
    : d(std::make_unique<Data>())
{
    d->levels = std::clamp(levels, 0, 12);
    d->unit = rootCell / (double)(1 << d->levels);
    const double rootSize = rootCell * Cells;
    for (int k = 0; k < 3; k++) {
        d->roots[k] = std::max(1, (int)std::ceil((hi[k] - lo[k]) / rootSize - 1e-9));
        // centered on the box
        d->origin[k] = 0.5 * (lo[k] + hi[k]) - 0.5 * d->roots[k] * rootSize;
    }
    d->requests.assign(std::max(1, threads), std::vector<std::vector<uint64_t>>(d->levels + 1));
}

Layout::Builder::~Builder() = default;

int Layout::Builder::Levels() const
{
    return d->levels;
}

double Layout::Builder::CellSize(int level) const
{
    return d->unit * (double)(1 << (d->levels - level));
}

void Layout::Builder::AddCapsule(int thread, const double a[3], const double b[3], double radius, int level)
{
    level = std::min(level, d->levels);
    if (level <= 0) {
        return;
    }
    // the bricks of the level in the box of the capsule
    const double size = CellSize(level) * Cells;
    const int count = 1 << level;
    int lo[3], hi[3];
    for (int k = 0; k < 3; k++) {
        const double l = std::min(a[k], b[k]) - radius, h = std::max(a[k], b[k]) + radius;
        lo[k] = std::max(0, (int)std::floor((l - d->origin[k]) / size));
        hi[k] = std::min(d->roots[k] * count - 1, (int)std::floor((h - d->origin[k]) / size));
        if (lo[k] > hi[k]) {
            return;
        }
    }
    const double half = 0.8660254037844386 * size;
    std::vector<uint64_t>& out = d->requests[thread][level];
    for (int z = lo[2]; z <= hi[2]; z++) {
        for (int y = lo[1]; y <= hi[1]; y++) {
            for (int x = lo[0]; x <= hi[0]; x++) {
                const double c[3] = {
                    d->origin[0] + (x + 0.5) * size,
                    d->origin[1] + (y + 0.5) * size,
                    d->origin[2] + (z + 0.5) * size,
                };
                if (BoxSegmentDistance(c, half, a, b) <= radius) {
                    out.push_back(Key(x, y, z));
                }
            }
        }
    }
}

std::shared_ptr<Layout> Layout::Builder::Build(ThreadPool& pool)
{
    auto layout = std::make_shared<Layout>();
    Layout& L = *layout;
    L.mLevels = d->levels;
    L.mUnit = d->unit;
    for (int k = 0; k < 3; k++) {
        L.mOrigin[k] = d->origin[k];
        L.mRoots[k] = d->roots[k];
        L.mExtent[k] = d->roots[k] * L.Size(0);
    }
    const int levels = d->levels;

    // the requested bricks of every level, merged
    std::vector<std::vector<uint64_t>> requested(levels + 1);
    {
        std::atomic<int> next {1};
        pool.Run([&](int) {
            for (int level; (level = next++) <= levels;) {
                std::vector<uint64_t>& out = requested[level];
                for (auto& perThread : d->requests) {
                    out.insert(out.end(), perThread[level].begin(), perThread[level].end());
                    std::vector<uint64_t>().swap(perThread[level]);
                }
                std::sort(out.begin(), out.end());
                out.erase(std::unique(out.begin(), out.end()), out.end());
            }
        });
    }

    // Split nodes by level, from the finest up: the parents of requested bricks and of split
    // nodes, and for 2:1 balance, the parents of the neighbors of split nodes
    std::vector<std::vector<uint64_t>> split(levels + 1);
    for (int level = levels; level >= 1; level--) {
        std::vector<uint64_t>& out = split[level - 1];
        for (uint64_t k : requested[level]) {
            out.push_back(ParentKey(k));
        }
        const int count = 1 << level;
        for (uint64_t k : split[level]) {
            int x, y, z;
            Unkey(k, x, y, z);
            for (int dz = -1; dz <= 1; dz++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        const int nx = x + dx, ny = y + dy, nz = z + dz;
                        if (nx < 0 || ny < 0 || nz < 0 || nx >= L.mRoots[0] * count
                            || ny >= L.mRoots[1] * count || nz >= L.mRoots[2] * count) {
                            continue;
                        }
                        out.push_back(Key(nx >> 1, ny >> 1, nz >> 1));
                    }
                }
            }
        }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        std::vector<uint64_t>().swap(requested[level]);
    }
    auto isSplit = [&](int level, int x, int y, int z) {
        const std::vector<uint64_t>& s = split[level];
        return std::binary_search(s.begin(), s.end(), Key(x, y, z));
    };

    // the tree, root by root, depth first
    const int roots = L.mRoots[0] * L.mRoots[1] * L.mRoots[2];
    L.mNodes.assign(roots, Node());
    for (int z = 0; z < L.mRoots[2]; z++) {
        for (int y = 0; y < L.mRoots[1]; y++) {
            for (int x = 0; x < L.mRoots[0]; x++) {
                Node& n = L.mNodes[((size_t)z * L.mRoots[1] + y) * L.mRoots[0] + x];
                n.level = 0;
                n.lo[0] = x * L.Size(0);
                n.lo[1] = y * L.Size(0);
                n.lo[2] = z * L.Size(0);
            }
        }
    }
    std::vector<int> stack;
    for (int r = 0; r < roots; r++) {
        stack.push_back(r);
        while (!stack.empty()) {
            const int node = stack.back();
            stack.pop_back();
            const Node parent = L.mNodes[node];
            const int size = L.Size(parent.level);
            if (parent.level >= levels
                || !isSplit(parent.level, parent.lo[0] / size, parent.lo[1] / size, parent.lo[2] / size)) {
                continue;
            }
            const int first = (int)L.mNodes.size();
            const int half = size / 2;
            for (int c = 0; c < 8; c++) {
                Node n;
                n.level = parent.level + 1;
                n.lo[0] = parent.lo[0] + (c & 1 ? half : 0);
                n.lo[1] = parent.lo[1] + (c & 2 ? half : 0);
                n.lo[2] = parent.lo[2] + (c & 4 ? half : 0);
                L.mNodes.push_back(n);
                stack.push_back(first + c);
            }
            L.mNodes[node].child = first;
        }
    }

    // leaves in depth first order; mesh units are the leaves of subtrees of at most 8 leaves.
    // Children are created after their parents, so a reverse pass counts leaves bottom up.
    std::vector<int> below(L.mNodes.size(), 0);
    for (int i = (int)L.mNodes.size() - 1; i >= 0; i--) {
        if (L.mNodes[i].child < 0) {
            below[i] = 1;
        }
        else {
            for (int c = 0; c < 8; c++) {
                below[i] += below[L.mNodes[i].child + c];
            }
        }
    }
    L.mUnits.assign(1, 0);
    std::vector<int> sub;
    for (int r = 0; r < roots; r++) {
        stack.push_back(r);
        while (!stack.empty()) {
            const int node = stack.back();
            stack.pop_back();
            if (L.mNodes[node].child >= 0 && below[node] > 8) {
                for (int c = 7; c >= 0; c--) {
                    stack.push_back(L.mNodes[node].child + c);
                }
                continue;
            }
            sub.assign(1, node);
            while (!sub.empty()) {
                const int s = sub.back();
                sub.pop_back();
                Node& m = L.mNodes[s];
                if (m.child >= 0) {
                    for (int c = 7; c >= 0; c--) {
                        sub.push_back(m.child + c);
                    }
                    continue;
                }
                m.leaf = (int)L.mLeaves.size();
                L.mLeaves.push_back({m.level, {m.lo[0], m.lo[1], m.lo[2]}, 1 << (levels - m.level)});
            }
            L.mUnits.push_back((int)L.mLeaves.size());
        }
    }
    L.mUnitOf.assign(L.mLeaves.size(), 0);
    for (int u = 0; u + 1 < (int)L.mUnits.size(); u++) {
        for (int l = L.mUnits[u]; l < L.mUnits[u + 1]; l++) {
            L.mUnitOf[l] = u;
        }
    }
    return layout;
}

bool Layout::UnitRange(const double lo[3], const double hi[3], int ilo[3], int ihi[3]) const
{
    for (int k = 0; k < 3; k++) {
        ilo[k] = std::max(0, (int)std::floor((lo[k] - mOrigin[k]) / mUnit));
        ihi[k] = std::min(mExtent[k] - 1, (int)std::floor((hi[k] - mOrigin[k]) / mUnit));
        if (ilo[k] > ihi[k]) {
            return false;
        }
    }
    return true;
}

int Layout::FindNode(int x, int y, int z) const
{
    if (x < 0 || y < 0 || z < 0 || x >= mExtent[0] || y >= mExtent[1] || z >= mExtent[2]) {
        return -1;
    }
    const int rs = Size(0);
    int node = ((z / rs) * mRoots[1] + y / rs) * mRoots[0] + x / rs;
    while (mNodes[node].child >= 0) {
        const Node& n = mNodes[node];
        const int half = Size(n.level + 1);
        const int c = (x - n.lo[0] >= half ? 1 : 0) | (y - n.lo[1] >= half ? 2 : 0)
            | (z - n.lo[2] >= half ? 4 : 0);
        node = n.child + c;
    }
    return node;
}

int Layout::FindLeaf(int x, int y, int z) const
{
    const int node = FindNode(x, y, z);
    return node < 0 ? -1 : mNodes[node].leaf;
}

void Layout::LeavesInBox(const double lo[3], const double hi[3], std::vector<int>& out) const
{
    LeavesWhere(lo, hi, [](const double*, double) { return true; }, out);
}

}  // namespace CAMSimulator::Material
