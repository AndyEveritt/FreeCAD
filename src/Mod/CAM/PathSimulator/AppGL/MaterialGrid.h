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

#include "MaterialLayout.h"
#include "MaterialTool.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{
class ThreadPool;
}

namespace CAMSimulator::Material
{

// Material along a dexel line from lo to hi (world coordinate along the line), with the packed
// normals of the surface at both ends
struct Span
{
    float lo;
    float hi;
    uint32_t nlo;
    uint32_t nhi;
};

// The dexels of a brick: lines along each axis through its grid points (see LineIndex). Lines
// extend one cell beyond the brick at both ends, so material crossing its border has no ends
// there.
struct Brick
{
    // a brick holds at most MaxSpans spans
    static constexpr size_t MaxSpans = 65535;
    std::vector<Span> spans;
    uint16_t start[BrickLines + 1] = {};
};

// The material a sweep removes, or a probe sweep meets, measured by Grid::Cut: its volume (the
// lengths along the lines of the three axes times their cells, averaged) and its centroid
struct CutStats
{
    double volume = 0;
    double moment[3] = {0, 0, 0};  // the centroid times the volume
};

// Stock material as tri-dexels in the bricks of a layout. Bricks are shared by copies of the grid
// (snapshots) and copied when they change. Empty bricks hold nothing, completely filled bricks
// share one marker.
class Grid
{
public:
    using BrickRef = std::shared_ptr<const Brick>;
    using Snapshot = std::vector<BrickRef>;

    void Init(std::shared_ptr<const Layout> layout);
    const Layout& GetLayout() const
    {
        return *mLayout;
    }
    const std::shared_ptr<const Layout>& LayoutRef() const
    {
        return mLayout;
    }

    // Material inside a closed triangle mesh
    void SetMesh(const std::vector<float>& xyz, const std::vector<uint32_t>& triangles, ThreadPool& pool);

    // Remove the material swept by the sweeps. Returns the work done (lines times sweeps). The
    // bricks the cut replaces are appended to 'undo' if given. Sweeps whose 'probes' flag is set
    // remove nothing, they only measure the material they meet. 'stats' gets what every sweep
    // removed or met, in the order of the sweeps.
    long Cut(
        const std::vector<const Sweep*>& sweeps,
        ThreadPool& pool,
        std::vector<std::pair<int, BrickRef>>* undo = nullptr,
        const std::vector<uint8_t>* probes = nullptr,
        std::vector<CutStats>* stats = nullptr
    );

    // Spans of a line of a leaf. 'scratch' holds the span of a completely filled brick.
    int Line(int leaf, int line, const Span*& out, Span& scratch) const;
    bool IsEmpty(int leaf) const
    {
        return !mBricks[leaf];
    }
    bool IsSolid(int leaf) const
    {
        return mBricks[leaf] == Solid();
    }
    // Extent of the lines along an axis of a leaf, world coordinates
    void LineRange(int leaf, int axis, double& lo, double& hi) const;
    // Origin of a line: the world position of its grid point, its coordinate along the axis is
    // that of the corner of the leaf
    void LineOrigin(int leaf, int line, double origin[3]) const;

    const Snapshot& Bricks() const
    {
        return mBricks;
    }
    const BrickRef& GetBrick(int leaf) const
    {
        return mBricks[leaf];
    }
    void SetBrick(int leaf, BrickRef brick)
    {
        if (mBricks[leaf] != brick) {
            mBricks[leaf] = std::move(brick);
            mDirty[leaf] = 1;
        }
    }
    // Replace the material with a snapshot, leaves that change become dirty
    void Restore(const Snapshot& snapshot);
    // Leaves changed since the last call
    std::vector<int> TakeDirty();
    void MarkAllDirty();

    // the marker of completely filled bricks
    static const BrickRef& Solid();

private:
    // Cut a leaf with the sweeps of the list, in order, returns the work done
    long CutLeaf(
        int leaf,
        const std::vector<const Sweep*>& sweeps,
        const int* list,
        int n,
        std::pair<int, BrickRef>* undo,
        const uint8_t* probes,
        std::atomic<double>* stats
    );

    std::shared_ptr<const Layout> mLayout;
    Snapshot mBricks;
    std::vector<uint8_t> mDirty;
};

}  // namespace CAMSimulator::Material
