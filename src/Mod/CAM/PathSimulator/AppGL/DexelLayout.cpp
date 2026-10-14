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


#include "DexelLayout.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

namespace
{

// Leaves of a mesh unit at most
constexpr int MaxUnitLeaves = 16;

float DistanceToSegment2(const float p[3], const float a[3], const float b[3], int dims)
{
    float ab[3], ap[3];
    float len2 = 0, dot = 0;
    for (int k = 0; k < dims; k++) {
        ab[k] = b[k] - a[k];
        ap[k] = p[k] - a[k];
        len2 += ab[k] * ab[k];
        dot += ab[k] * ap[k];
    }
    const float t = len2 > 0 ? std::clamp(dot / len2, 0.0f, 1.0f) : 0.0f;
    float d2 = 0;
    for (int k = 0; k < dims; k++) {
        const float d = ap[k] - t * ab[k];
        d2 += d * d;
    }
    return d2;
}

}  // namespace

void LayoutRegion::UpdateBounds()
{
    for (int k = 0; k < 3; k++) {
        lo[k] = std::min(a[k], b[k]) - radius;
        hi[k] = std::max(a[k], b[k]) + radius;
    }
    if (vertical) {
        lo[2] = zlo;
        hi[2] = zhi;
    }
}

void DexelLayout::Init(const float lo[3], const float hi[3], float fineCell, int maxLevel)
{
    mFineCell = fineCell;
    mMaxLevel = maxLevel;
    mNodes.clear();
    mLeaves.clear();
    mUnits.clear();
    mUnitChunk.clear();
    mAffectedUnits.clear();

    const double rootSize = (double)Size(0) * fineCell;
    for (int k = 0; k < 3; k++) {
        // a margin of a cell around the box, and an offset so that faces of the box at round
        // coordinates are not on grid points
        const double extent = (double)hi[k] - lo[k] + 2.0 * fineCell;
        mRoots[k] = std::max(1, (int)std::ceil(extent / rootSize));
        mOrigin[k] = ((double)lo[k] + hi[k]) / 2 - mRoots[k] * rootSize / 2 + 0.3183 * fineCell;
        if (mOrigin[k] > lo[k] - fineCell / 2) {
            mOrigin[k] -= fineCell;
        }
    }
    mRootCount = mRoots[0] * mRoots[1] * mRoots[2];
    mNodes.reserve(mRootCount);
    for (int z = 0; z < mRoots[2]; z++) {
        for (int y = 0; y < mRoots[1]; y++) {
            for (int x = 0; x < mRoots[0]; x++) {
                Node node {};
                node.child = -1;
                node.leaf = -1;
                node.origin[0] = x * Size(0);
                node.origin[1] = y * Size(0);
                node.origin[2] = z * Size(0);
                mNodes.push_back(node);
            }
        }
    }
}

int DexelLayout::FineIndex(int axis, float x) const
{
    return (int)std::floor((x - mOrigin[axis]) / mFineCell);
}

bool DexelLayout::Intersects(const Node& node, const LayoutRegion& region) const
{
    const int size = Size(node.level);
    float lo[3], hi[3], center[3];
    for (int k = 0; k < 3; k++) {
        lo[k] = Coord(k, node.origin[k]);
        hi[k] = Coord(k, node.origin[k] + size);
        if (hi[k] < region.lo[k] || lo[k] > region.hi[k]) {
            return false;
        }
        center[k] = (lo[k] + hi[k]) / 2;
    }
    const float half = (hi[0] - lo[0]) / 2;
    if (region.vertical) {
        const float reach = region.radius + half * 1.4143f;
        return DistanceToSegment2(center, region.a, region.b, 2) <= reach * reach;
    }
    const float reach = region.radius + half * 1.7321f;
    return DistanceToSegment2(center, region.a, region.b, 3) <= reach * reach;
}

void DexelLayout::Split(int index)
{
    const int level = mNodes[index].level + 1;
    const int half = Size(level);
    const int child = (int)mNodes.size();
    for (int i = 0; i < 8; i++) {
        Node node {};
        node.child = -1;
        node.leaf = -1;
        node.level = (int8_t)level;
        node.minLevel = node.maxLevel = (int8_t)level;
        for (int k = 0; k < 3; k++) {
            node.origin[k] = mNodes[index].origin[k] + ((i >> k) & 1) * half;
        }
        mNodes.push_back(node);
    }
    mNodes[index].child = child;
}

void DexelLayout::UpdateLevels(int index)
{
    Node& node = mNodes[index];
    if (node.child < 0) {
        node.minLevel = node.maxLevel = node.level;
        return;
    }
    int8_t lo = 127, hi = 0;
    for (int i = 0; i < 8; i++) {
        lo = std::min(lo, mNodes[node.child + i].minLevel);
        hi = std::max(hi, mNodes[node.child + i].maxLevel);
    }
    node.minLevel = lo;
    node.maxLevel = hi;
}

void DexelLayout::RefineNode(int index, const LayoutRegion& region, int level)
{
    if (mNodes[index].minLevel >= level || !Intersects(mNodes[index], region)) {
        return;
    }
    if (mNodes[index].child < 0) {
        Split(index);
    }
    const int child = mNodes[index].child;
    for (int i = 0; i < 8; i++) {
        RefineNode(child + i, region, level);
    }
    UpdateLevels(index);
}

void DexelLayout::Refine(const LayoutRegion& region, int level)
{
    level = std::min(level, mMaxLevel);
    if (level <= 0) {
        return;
    }
    const int size = Size(0);
    int r0[3], r1[3];
    for (int k = 0; k < 3; k++) {
        r0[k] = std::max(0, FineIndex(k, region.lo[k]) / size);
        r1[k] = std::min(mRoots[k] - 1, FineIndex(k, region.hi[k]) / size);
        if (FineIndex(k, region.hi[k]) < 0 || r0[k] > r1[k]) {
            return;
        }
    }
    for (int z = r0[2]; z <= r1[2]; z++) {
        for (int y = r0[1]; y <= r1[1]; y++) {
            for (int x = r0[0]; x <= r1[0]; x++) {
                RefineNode(x + mRoots[0] * (y + mRoots[1] * z), region, level);
            }
        }
    }
}

int DexelLayout::FindNode(int fx, int fy, int fz) const
{
    const int size = Size(0);
    if (fx < 0 || fy < 0 || fz < 0) {
        return -1;
    }
    const int rx = fx / size, ry = fy / size, rz = fz / size;
    if (rx >= mRoots[0] || ry >= mRoots[1] || rz >= mRoots[2]) {
        return -1;
    }
    int index = rx + mRoots[0] * (ry + mRoots[1] * rz);
    while (mNodes[index].child >= 0) {
        const Node& node = mNodes[index];
        const int half = Size(node.level + 1);
        const int octant = (fx >= node.origin[0] + half ? 1 : 0) | (fy >= node.origin[1] + half ? 2 : 0)
            | (fz >= node.origin[2] + half ? 4 : 0);
        index = node.child + octant;
    }
    return index;
}

int DexelLayout::FindLeaf(int fx, int fy, int fz) const
{
    const int node = FindNode(fx, fy, fz);
    return node < 0 ? -1 : mNodes[node].leaf;
}

void DexelLayout::Finish()
{
    // balance: neighbors differ by at most one level
    std::vector<int> work;
    for (int i = 0; i < (int)mNodes.size(); i++) {
        if (mNodes[i].child < 0) {
            work.push_back(i);
        }
    }
    while (!work.empty()) {
        const int index = work.back();
        work.pop_back();
        if (mNodes[index].child >= 0 || mNodes[index].level < 2) {
            continue;
        }
        const int level = mNodes[index].level;
        const int size = Size(level);
        bool split = false;
        for (int d = 0; d < 27; d++) {
            const int dx = d % 3 - 1, dy = d / 3 % 3 - 1, dz = d / 9 - 1;
            if (dx == 0 && dy == 0 && dz == 0) {
                continue;
            }
            // copied, splitting reallocates the nodes
            const int o[3] = {mNodes[index].origin[0], mNodes[index].origin[1], mNodes[index].origin[2]};
            const int sample[3] = {
                dx < 0 ? o[0] - 1 : dx > 0 ? o[0] + size : o[0] + size / 2,
                dy < 0 ? o[1] - 1 : dy > 0 ? o[1] + size : o[1] + size / 2,
                dz < 0 ? o[2] - 1 : dz > 0 ? o[2] + size : o[2] + size / 2,
            };
            const int neighbor = FindNode(sample[0], sample[1], sample[2]);
            if (neighbor >= 0 && mNodes[neighbor].level < level - 1) {
                Split(neighbor);
                for (int i = 0; i < 8; i++) {
                    work.push_back(mNodes[neighbor].child + i);
                }
                split = true;
            }
        }
        if (split) {
            work.push_back(index);
        }
    }

    // levels of the subtrees, children are after their parents
    for (int i = (int)mNodes.size() - 1; i >= 0; i--) {
        UpdateLevels(i);
    }

    // leaves and mesh units, a chunk per root
    mLeaves.clear();
    mUnits.clear();
    mUnitChunk.clear();
    for (int root = 0; root < mRootCount; root++) {
        CollectLeaves(root, root);
        AssignUnits(root, root);
    }

    // the leaves touching each leaf, found at both halves of every face and edge, where finer
    // neighbors can be
    std::vector<std::vector<int>>& neighbors = mNeighbors;
    neighbors.assign(mLeaves.size(), {});
    for (int i = 0; i < (int)mLeaves.size(); i++) {
        const LayoutLeaf& leaf = mLeaves[i];
        const int size = Size(leaf.level);
        std::vector<int>& list = neighbors[i];
        for (int d = 0; d < 27; d++) {
            const int dir[3] = {d % 3 - 1, d / 3 % 3 - 1, d / 9 - 1};
            if (dir[0] == 0 && dir[1] == 0 && dir[2] == 0) {
                continue;
            }
            for (int s = 0; s < 8; s++) {
                int p[3];
                bool skip = false;
                for (int k = 0; k < 3; k++) {
                    const int o = leaf.origin[k];
                    if (dir[k] < 0) {
                        p[k] = o - 1;
                        skip |= ((s >> k) & 1) != 0;
                    }
                    else if (dir[k] > 0) {
                        p[k] = o + size;
                        skip |= ((s >> k) & 1) != 0;
                    }
                    else {
                        p[k] = o + (((s >> k) & 1) ? size * 3 / 4 : size / 4);
                    }
                }
                if (skip) {
                    continue;
                }
                const int neighbor = FindLeaf(p[0], p[1], p[2]);
                if (neighbor >= 0) {
                    list.push_back(neighbor);
                }
            }
        }
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }

    // units depending on each leaf: its own and those of the leaves touching it. The vertex of a
    // coarser neighbor without a surface of its own comes from the crossings on its faces, it
    // depends on all leaves touching that neighbor, so their units depend on the leaf too.
    mAffectedUnits.assign(mLeaves.size(), {});
    for (int i = 0; i < (int)mLeaves.size(); i++) {
        std::vector<int>& units = mAffectedUnits[i];
        units.push_back(mLeaves[i].unit);
        for (int n : neighbors[i]) {
            units.push_back(mLeaves[n].unit);
            if (mLeaves[n].level < mLeaves[i].level) {
                for (int m : neighbors[n]) {
                    units.push_back(mLeaves[m].unit);
                }
            }
        }
        std::sort(units.begin(), units.end());
        units.erase(std::unique(units.begin(), units.end()), units.end());
    }
    mUnitDependencies.assign(mUnits.size(), {});
    for (int i = 0; i < (int)mLeaves.size(); i++) {
        for (int unit : mAffectedUnits[i]) {
            mUnitDependencies[unit].push_back(i);
        }
    }
}

void DexelLayout::CollectLeaves(int index, int chunk)
{
    Node& node = mNodes[index];
    if (node.child >= 0) {
        for (int i = 0; i < 8; i++) {
            CollectLeaves(mNodes[index].child + i, chunk);
        }
        return;
    }
    node.leaf = (int)mLeaves.size();
    LayoutLeaf leaf {};
    leaf.level = node.level;
    leaf.step = 1 << (mMaxLevel - node.level);
    for (int k = 0; k < 3; k++) {
        leaf.origin[k] = node.origin[k];
    }
    leaf.node = index;
    leaf.unit = -1;
    mLeaves.push_back(leaf);
}

void DexelLayout::AssignUnits(int index, int chunk)
{
    // leaves of the subtree, they are consecutive
    int first = -1, last = -1;
    std::vector<int> stack {index};
    while (!stack.empty()) {
        const Node& node = mNodes[stack.back()];
        stack.pop_back();
        if (node.child < 0) {
            first = first < 0 ? node.leaf : std::min(first, node.leaf);
            last = std::max(last, node.leaf);
            continue;
        }
        for (int i = 0; i < 8; i++) {
            stack.push_back(node.child + i);
        }
    }
    if (last - first + 1 > MaxUnitLeaves && mNodes[index].child >= 0) {
        for (int i = 0; i < 8; i++) {
            AssignUnits(mNodes[index].child + i, chunk);
        }
        return;
    }
    const int unit = (int)mUnits.size();
    mUnits.emplace_back();
    mUnitChunk.push_back(chunk);
    for (int leaf = first; leaf <= last; leaf++) {
        mUnits.back().push_back(leaf);
        mLeaves[leaf].unit = unit;
    }
}

template<class Fn>
void DexelLayout::Visit(int index, const LayoutRegion& region, Fn&& fn) const
{
    const Node& node = mNodes[index];
    if (!Intersects(node, region)) {
        return;
    }
    if (node.child < 0) {
        fn(node);
        return;
    }
    for (int i = 0; i < 8; i++) {
        Visit(node.child + i, region, fn);
    }
}

void DexelLayout::LeavesIn(const LayoutRegion& region, std::vector<int>& out) const
{
    out.clear();
    const int size = Size(0);
    int r0[3], r1[3];
    for (int k = 0; k < 3; k++) {
        const int a = FineIndex(k, region.lo[k]), b = FineIndex(k, region.hi[k]);
        if (b < 0) {
            return;
        }
        r0[k] = std::max(0, a / size);
        r1[k] = std::min(mRoots[k] - 1, b / size);
        if (r0[k] > r1[k]) {
            return;
        }
    }
    for (int z = r0[2]; z <= r1[2]; z++) {
        for (int y = r0[1]; y <= r1[1]; y++) {
            for (int x = r0[0]; x <= r1[0]; x++) {
                Visit(x + mRoots[0] * (y + mRoots[1] * z), region, [&](const Node& node) {
                    out.push_back(node.leaf);
                });
            }
        }
    }
}

int DexelLayout::MaxLevelIn(const LayoutRegion& region) const
{
    const int size = Size(0);
    int r0[3], r1[3];
    for (int k = 0; k < 3; k++) {
        const int a = FineIndex(k, region.lo[k]), b = FineIndex(k, region.hi[k]);
        if (b < 0) {
            return -1;
        }
        r0[k] = std::max(0, a / size);
        r1[k] = std::min(mRoots[k] - 1, b / size);
        if (r0[k] > r1[k]) {
            return -1;
        }
    }
    int best = -1;
    std::vector<int> stack;
    for (int z = r0[2]; z <= r1[2]; z++) {
        for (int y = r0[1]; y <= r1[1]; y++) {
            for (int x = r0[0]; x <= r1[0]; x++) {
                stack.push_back(x + mRoots[0] * (y + mRoots[1] * z));
            }
        }
    }
    while (!stack.empty()) {
        const Node& node = mNodes[stack.back()];
        stack.pop_back();
        if (node.maxLevel <= best || !Intersects(node, region)) {
            continue;
        }
        if (node.child < 0) {
            best = node.level;
            continue;
        }
        for (int i = 0; i < 8; i++) {
            stack.push_back(node.child + i);
        }
    }
    return best;
}

}  // namespace CAMSimulator
