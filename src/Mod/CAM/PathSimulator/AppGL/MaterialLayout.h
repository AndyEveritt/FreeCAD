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

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{
class ThreadPool;
}

namespace CAMSimulator::Material
{

// Cells per edge of a brick, and its dexel lines: along each axis through all (Cells + 1)^2 grid
// points of the brick, including those on its border
constexpr int Cells = 8;
constexpr int AxisLines = (Cells + 1) * (Cells + 1);
constexpr int BrickLines = 3 * AxisLines;

// Index of the line along 'axis' through grid point (i, j) of the other two axes, i along
// (axis + 1) % 3 and j along (axis + 2) % 3
inline int LineIndex(int axis, int i, int j)
{
    return axis * AxisLines + j * (Cells + 1) + i;
}

// Adaptive resolution of the stock: an octree of bricks of Cells^3 cells over a grid of root
// bricks. The bricks of level k have cells of half the size of level k - 1. Positions are given
// in units of the cells of the finest level ("units"). Neighboring bricks (sharing a face, an
// edge or a corner) differ by at most one level.
class Layout
{
public:
    struct Leaf
    {
        int level;
        int lo[3];  // corner in units
        int cell;   // cell size in units
    };

    // Collects where the layout needs which level, from several threads, and builds it
    class Builder
    {
    public:
        // Covers the box with root bricks of cells of size rootCell, with 'levels' finer levels
        Builder(const double lo[3], const double hi[3], double rootCell, int levels, int threads);
        ~Builder();
        // Bricks within 'radius' of the segment [a, b] get at least 'level'
        void AddCapsule(int thread, const double a[3], const double b[3], double radius, int level);
        std::shared_ptr<Layout> Build(ThreadPool& pool);

        int Levels() const;
        double CellSize(int level) const;

    private:
        struct Data;
        std::unique_ptr<Data> d;
    };

    int Levels() const
    {
        return mLevels;
    }
    // Cell size of a level in world units
    double CellSize(int level) const
    {
        return mUnit * (double)(1 << (mLevels - level));
    }
    double Unit() const
    {
        return mUnit;
    }
    const double* Origin() const
    {
        return mOrigin;
    }
    // World coordinate of a position in units
    double Coord(int axis, long units) const
    {
        return mOrigin[axis] + (double)units * mUnit;
    }
    // Size of the covered box in units
    const int* Extent() const
    {
        return mExtent;
    }

    int LeafCount() const
    {
        return (int)mLeaves.size();
    }
    const Leaf& GetLeaf(int leaf) const
    {
        return mLeaves[leaf];
    }
    // Leaf containing the point in units (the cell from it towards positive), -1 outside
    int FindLeaf(int x, int y, int z) const;
    // Leaves intersecting a box in world coordinates, appended
    void LeavesInBox(const double lo[3], const double hi[3], std::vector<int>& out) const;

    // Leaves in a box for which mayTouch(center, halfDiagonal) of their nodes holds, appended
    template<class F>
    void LeavesWhere(const double lo[3], const double hi[3], const F& mayTouch, std::vector<int>& out) const
    {
        int ilo[3], ihi[3];
        if (!UnitRange(lo, hi, ilo, ihi)) {
            return;
        }
        const int rs = Size(0);
        thread_local std::vector<int> stack;
        stack.clear();
        for (int z = ilo[2] / rs; z <= ihi[2] / rs; z++) {
            for (int y = ilo[1] / rs; y <= ihi[1] / rs; y++) {
                for (int x = ilo[0] / rs; x <= ihi[0] / rs; x++) {
                    stack.push_back((z * mRoots[1] + y) * mRoots[0] + x);
                }
            }
        }
        while (!stack.empty()) {
            const int node = stack.back();
            stack.pop_back();
            const Node& n = mNodes[node];
            const int size = Size(n.level);
            double c[3];
            bool overlaps = true;
            for (int k = 0; k < 3; k++) {
                overlaps = overlaps && n.lo[k] <= ihi[k] && n.lo[k] + size > ilo[k];
                c[k] = Coord(k, n.lo[k]) + 0.5 * size * mUnit;
            }
            if (!overlaps || !mayTouch(c, 0.8660254037844386 * size * mUnit)) {
                continue;
            }
            if (n.child < 0) {
                out.push_back(n.leaf);
                continue;
            }
            for (int k = 0; k < 8; k++) {
                stack.push_back(n.child + k);
            }
        }
    }

    // Mesh units: groups of up to 8 leaves of a subtree, whose meshes are built together
    int UnitOf(int leaf) const
    {
        return mUnitOf[leaf];
    }
    int UnitCount() const
    {
        return (int)mUnits.size() - 1;
    }
    // Leaves of a unit, from UnitBegin to UnitEnd
    int UnitBegin(int unit) const
    {
        return mUnits[unit];
    }
    int UnitEnd(int unit) const
    {
        return mUnits[unit + 1];
    }

private:
    struct Node
    {
        int child = -1;  // first of 8 children
        int leaf = -1;
        int level = 0;
        int lo[3] = {0, 0, 0};
    };
    int Size(int level) const
    {
        return Cells << (mLevels - level);
    }
    int FindNode(int x, int y, int z) const;
    // the units of a box, clamped to the layout, false if outside
    bool UnitRange(const double lo[3], const double hi[3], int ilo[3], int ihi[3]) const;

    double mOrigin[3] = {};
    double mUnit = 1;
    int mLevels = 0;
    int mRoots[3] = {0, 0, 0};
    int mExtent[3] = {0, 0, 0};
    std::vector<Node> mNodes;  // the roots first
    std::vector<Leaf> mLeaves;
    std::vector<int> mUnitOf;
    std::vector<int> mUnits;
};

}  // namespace CAMSimulator::Material
