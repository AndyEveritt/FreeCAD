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

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{

class ThreadPool;

constexpr int DexelTileBits = 4;
constexpr int DexelTileSize = 1 << DexelTileBits;
constexpr int DexelTileCells = DexelTileSize * DexelTileSize;

// Material of a square of DexelTileSize x DexelTileSize dexels. Every dexel is a vertical line
// through a cell center, holding the sorted, disjoint height intervals where there is material.
struct DexelTile
{
    // 0..2: number of intervals stored in 'inline', 3: intervals stored in 'large'
    uint8_t count[DexelTileCells] = {};
    Interval inlineIntervals[DexelTileCells][2];
    std::vector<std::vector<Interval>> large;  // empty, or one vector per cell

    // Frozen tiles are shared (by snapshots) and never modified, they are copied instead.
    bool frozen = false;
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

// Stock material as a grid of vertical dexels (Z-dexels), stored in tiles. Tiles are shared
// between copies of the stock and copied when modified, so snapshots of the stock are cheap.
class DexelStock
{
public:
    using TileRef = std::shared_ptr<DexelTile>;
    using Snapshot = std::vector<TileRef>;

    // Grid with nx * ny cells of size 'cell', cell (0, 0) starting at (x0, y0). No material.
    void Init(float x0, float y0, float cell, int nx, int ny);

    // Material inside a closed triangle mesh. xyz: vertex positions, triangles: vertex indices
    void SetMesh(const std::vector<float>& xyz, const std::vector<uint32_t>& triangles, ThreadPool& pool);

    // Remove the material swept by the moves. Returns an estimate of the work done (cells).
    // Stops early when 'cancel' becomes true.
    long Cut(
        const std::vector<CutMove>& moves,
        ThreadPool& pool,
        const std::atomic<bool>* cancel = nullptr
    );

    // Number of material intervals of a cell, 'out' points to them
    int Cell(int ix, int iy, const Interval*& out) const;

    // Mark all tiles as shared, required before Tiles() can be used as a snapshot
    void Freeze();
    const Snapshot& Tiles() const
    {
        return mTiles;
    }
    // Replace the material with a snapshot of this stock, marking changed tiles as dirty
    void Restore(const Snapshot& snapshot);

    // Tiles changed since the last call
    std::vector<int> TakeDirtyTiles();
    // Bytes of tiles created by copying since the last call
    long TakeNewTileBytes();

    float X0() const
    {
        return mX0;
    }
    float Y0() const
    {
        return mY0;
    }
    float CellSize() const
    {
        return mCell;
    }
    int NX() const
    {
        return mNX;
    }
    int NY() const
    {
        return mNY;
    }
    int TilesX() const
    {
        return mTilesX;
    }
    int TilesY() const
    {
        return mTilesY;
    }

    // Estimated work of cutting a move, in cells
    long EstimateWork(const CutMove& move) const;

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

private:
    int TileIndex(int ix, int iy) const
    {
        return (iy >> DexelTileBits) * mTilesX + (ix >> DexelTileBits);
    }
    static int LocalIndex(int ix, int iy)
    {
        return ((iy & (DexelTileSize - 1)) << DexelTileBits) | (ix & (DexelTileSize - 1));
    }

    void CutWorker(const CutMove& move, int worker, int workers, long& work);
    void CutVerticalSegment(
        const CutterProfile& tool,
        const float a[3],
        const float b[3],
        int worker,
        int workers,
        long& work
    );
    void CutInstance(
        const CutterProfile& tool,
        const float tip[3],
        const float axis[3],
        int worker,
        int workers,
        long& work,
        std::vector<Interval>& scratch
    );
    bool SubtractCell(int tile, int local, const Interval* removal, int count);
    // Length of the tool from its tip that can reach the material during a move
    float CuttingLength(const CutMove& move) const;

    float mX0 = 0;
    float mY0 = 0;
    float mCell = 1;
    int mNX = 0;
    int mNY = 0;
    int mTilesX = 0;
    int mTilesY = 0;
    Snapshot mTiles;
    std::vector<uint8_t> mDirty;
    std::atomic<long> mNewTileBytes {0};
    float mTopZ = 1e30f;
};

}  // namespace CAMSimulator
