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

#include "CutterProfile.h"
#include "DexelLayout.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{

class ThreadPool;

// Unit normal of a material surface, packed into 30 bits, and a flag for faces created by cutting.
// A zero normal is packed as NoNormal: the surface was made by a sharp edge of a tool, its normal
// isn't known.
constexpr uint32_t NoNormal = 1u << 30;
uint32_t PackNormal(const float n[3], bool cut);
void UnpackNormal(uint32_t packed, float n[3]);
inline bool IsCutFace(uint32_t packed)
{
    return (packed >> 31) != 0;
}
inline bool HasNormal(uint32_t packed)
{
    return (packed & NoNormal) == 0;
}

// Material along a dexel line from lo to hi, in world coordinates along the line, with the
// normals of the material surface at both ends
struct Span
{
    float lo;
    float hi;
    uint32_t nlo;
    uint32_t nhi;
};

// Material of a brick of the layout, as dexels: lines along X, Y and Z through all grid points
// of the brick, including those on its border. Lines along axis 'a' are indexed by
// a * BrickAxisLines + j * BrickLines + i, where i and j count along the axes (a + 1) % 3 and
// (a + 2) % 3. Lines extend one cell beyond the brick at both ends, so the material at the
// border is known exactly in each brick.
struct Brick
{
    std::vector<Span> spans;
    uint32_t start[BrickLineCount + 1] = {};  // first span of each line
    // Frozen bricks are shared (by snapshots) and never modified, they are copied instead.
    bool frozen = false;
    bool solid = false;  // completely filled, no spans stored
};

// A tool motion: the tool tip moves linearly from p0 to p1, while its axis (unit vector from the
// tip towards the shank) turns from a0 to a1.
struct CutMove
{
    const CutterProfile* tool = nullptr;
    float p0[3];
    float p1[3];
    float a0[3];
    float a1[3];
};

// Stock material as tri-dexels in the bricks of an adaptive layout. Bricks are shared between
// copies of the stock and copied when modified, so snapshots of the stock are cheap. Empty bricks
// are not stored.
class DexelStock
{
public:
    using BrickRef = std::shared_ptr<Brick>;
    using Snapshot = std::vector<BrickRef>;

    // No material
    void Init(std::shared_ptr<const DexelLayout> layout);
    const DexelLayout& Layout() const
    {
        return *mLayout;
    }
    const std::shared_ptr<const DexelLayout>& LayoutRef() const
    {
        return mLayout;
    }

    // Material inside a closed triangle mesh. xyz: vertex positions, triangles: vertex indices
    void SetMesh(const std::vector<float>& xyz, const std::vector<uint32_t>& triangles, ThreadPool& pool);

    // Remove the material swept by the moves. Returns an estimate of the work done (lines).
    // Stops early when 'cancel' becomes true.
    long Cut(
        const std::vector<CutMove>& moves,
        ThreadPool& pool,
        const std::atomic<bool>* cancel = nullptr
    );
    // Estimated work of cutting a move, in lines
    long EstimateWork(const CutMove& move) const;

    // Material spans of a line of a leaf, 'out' points to them. 'scratch' holds the span of
    // completely filled bricks.
    int Line(int leaf, int line, const Span*& out, Span& scratch) const;
    bool IsEmpty(int leaf) const
    {
        return !mBricks[leaf];
    }
    bool IsSolid(int leaf) const
    {
        return mBricks[leaf] && mBricks[leaf]->solid;
    }
    // Range of the lines along an axis of a leaf
    void LineRange(int leaf, int axis, float& lo, float& hi) const;

    // Mark all bricks as shared, required before Bricks() can be used as a snapshot
    void Freeze();
    const Snapshot& Bricks() const
    {
        return mBricks;
    }
    // Replace the material with a snapshot of this stock, marking changed leaves as dirty
    void Restore(const Snapshot& snapshot);

    // Leaves changed since the last call
    std::vector<int> TakeDirtyLeaves();
    // Bytes of bricks created by copying since the last call
    long TakeNewBrickBytes();

    // Upper bound of the height of the material, parts of tools above it are not cut. Set by
    // SetMesh, unbounded otherwise.
    float TopZ() const
    {
        return mTopZ;
    }
    void SetTopZ(float z)
    {
        mTopZ = z;
    }

    // Length of the tool from its tip that can reach the material during a move
    float CuttingLength(const CutMove& move) const;

private:
    struct Prepared;
    struct InstanceRef;
    struct Changes;
    struct Scratch;

    void Prepare(const CutMove& move, Prepared& prepared) const;
    // Cut the lines along an axis of a brick, from line lineFrom to lineTo across, with all moves
    // touching the brick
    void CutBrick(
        int leaf,
        const Prepared* const* moves,
        int count,
        int axis,
        int lineFrom,
        int lineTo,
        Scratch& scratch,
        Changes& changes
    );
    // Replace the changed lines of a brick
    void Commit(int leaf, Changes* parts, int count, Scratch& scratch);

    std::shared_ptr<const DexelLayout> mLayout;
    Snapshot mBricks;
    std::vector<uint8_t> mDirty;
    std::atomic<long> mNewBrickBytes {0};
    float mTopZ = 1e30f;
};

}  // namespace CAMSimulator
