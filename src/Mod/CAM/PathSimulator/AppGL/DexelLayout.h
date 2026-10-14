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


#pragma once

#include <cstdint>
#include <vector>

namespace CAMSimulator
{

// Cells along each side of a brick
constexpr int BrickCells = 8;
// Grid lines along each side of a brick, the lines on its border are shared with its neighbors
constexpr int BrickLines = BrickCells + 1;
// Dexel lines of a brick along one axis, and in total
constexpr int BrickAxisLines = BrickLines * BrickLines;
constexpr int BrickLineCount = 3 * BrickAxisLines;

// A region that a tool may touch during a move: points within 'radius' of the segment a-b. For
// vertical tools the distance is measured in the XY plane and the region is limited to the
// heights [zlo, zhi].
struct LayoutRegion
{
    float a[3];
    float b[3];
    float radius;
    bool vertical;
    float zlo, zhi;
    // bounding box
    float lo[3];
    float hi[3];

    void UpdateBounds();
};

// A brick of the layout: BrickCells^3 cells, at a resolution level
struct LayoutLeaf
{
    int level;
    int origin[3];  // corner, in cells of the finest level
    int step;       // size of its cells, in cells of the finest level
    int node;
    int unit;  // mesh unit
};

// Adaptive resolution: the stock is covered by a regular grid of root bricks, at level 0. Each
// brick can be divided into 8 bricks of the next level, with cells of half the size, up to the
// finest level. Bricks are refined where tools cut, so the detail follows the tools. Neighboring
// bricks differ by at most one level.
//
// All coordinates are on the grid of the finest level, so every brick computes the world position
// of a grid point the same way.
class DexelLayout
{
public:
    // Root bricks covering the box [lo, hi], cells of 'fineCell' at the finest level
    // 'maxLevel', and 2^maxLevel times larger at level 0
    void Init(const float lo[3], const float hi[3], float fineCell, int maxLevel);
    // Refine the bricks intersecting the region to at least 'level'
    void Refine(const LayoutRegion& region, int level);
    // Balance the levels of neighbors and create the leaves and mesh units
    void Finish();

    int MaxLevel() const
    {
        return mMaxLevel;
    }
    float FineCell() const
    {
        return mFineCell;
    }
    float CellSize(int level) const
    {
        return mFineCell * (float)(1 << (mMaxLevel - level));
    }
    // World coordinate of a grid coordinate along an axis
    float Coord(int axis, int fine) const
    {
        return (float)(mOrigin[axis] + (double)fine * mFineCell);
    }
    // Grid coordinate at or below a world coordinate
    int FineIndex(int axis, float x) const;
    // Size of the layout along an axis, in cells of the finest level
    int GridCells(int axis) const
    {
        return mRoots[axis] * Size(0);
    }

    int LeafCount() const
    {
        return (int)mLeaves.size();
    }
    const LayoutLeaf& Leaf(int i) const
    {
        return mLeaves[i];
    }
    // Leaf containing the grid point (half open), or -1 outside the layout
    int FindLeaf(int fx, int fy, int fz) const;
    // Leaves that may intersect the region
    void LeavesIn(const LayoutRegion& region, std::vector<int>& out) const;
    // Finest level of the leaves that may intersect the region, -1 if none
    int MaxLevelIn(const LayoutRegion& region) const;

    // Mesh units: small groups of leaves that are meshed together. Units are grouped into
    // chunks for drawing, one per root brick.
    int UnitCount() const
    {
        return (int)mUnits.size();
    }
    const std::vector<int>& UnitLeaves(int unit) const
    {
        return mUnits[unit];
    }
    int UnitChunk(int unit) const
    {
        return mUnitChunk[unit];
    }
    int ChunkCount() const
    {
        return mRootCount;
    }
    // Units with polygons that depend on a leaf: the units of the leaf and of its neighbors
    const std::vector<int>& AffectedUnits(int leaf) const
    {
        return mAffectedUnits[leaf];
    }
    // Leaves the polygons of a unit depend on: its leaves and their neighbors
    const std::vector<int>& UnitDependencies(int unit) const
    {
        return mUnitDependencies[unit];
    }

private:
    struct Node
    {
        int child;  // first of 8 children, -1 for leaves
        int leaf;
        int8_t level;
        int8_t minLevel;  // coarsest and finest leaf level in the subtree
        int8_t maxLevel;
        int origin[3];
    };

    int Size(int level) const
    {
        return BrickCells << (mMaxLevel - level);
    }
    bool Intersects(const Node& node, const LayoutRegion& region) const;
    void Split(int node);
    void RefineNode(int node, const LayoutRegion& region, int level);
    void UpdateLevels(int node);
    int FindNode(int fx, int fy, int fz) const;
    template<class Fn>
    void Visit(int node, const LayoutRegion& region, Fn&& fn) const;
    void CollectLeaves(int node, int chunk);
    void AssignUnits(int node, int chunk);

    double mOrigin[3] = {};
    float mFineCell = 1;
    int mMaxLevel = 0;
    int mRoots[3] = {};
    int mRootCount = 0;
    std::vector<Node> mNodes;  // the roots first, x fastest
    std::vector<LayoutLeaf> mLeaves;
    std::vector<std::vector<int>> mUnits;
    std::vector<int> mUnitChunk;
    std::vector<std::vector<int>> mAffectedUnits;
    std::vector<std::vector<int>> mUnitDependencies;
};

}  // namespace CAMSimulator
