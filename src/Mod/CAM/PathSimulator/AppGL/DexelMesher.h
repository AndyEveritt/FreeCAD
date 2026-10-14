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

#include "DexelStock.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{

class ThreadPool;

// Vertex of the stock mesh: position and normalized normal
struct DexelVertex
{
    float x, y, z;
    int8_t nx, ny, nz, pad;
};

// Triangles of a part of the stock: faces of the original stock and cut faces
struct DexelMesh
{
    std::vector<DexelVertex> vertices;
    std::vector<uint32_t> stockIndices;
    std::vector<uint32_t> cutIndices;
};

struct LeafGeometry;

// Surface of the leaves of a stock, the input of the mesh: inside state of the grid points,
// crossings of the edges, cells merged into cubes and their vertices. Kept while the material of
// a leaf doesn't change.
class DexelMeshCache
{
public:
    DexelMeshCache();
    ~DexelMeshCache();

    void Reset(int leaves);
    void Invalidate(int leaf);
    // Compute the leaves that are missing, in parallel
    void Update(const DexelStock& stock, const std::vector<int>& leaves, ThreadPool& pool);
    // nullptr: no surface in the leaf, completely inside or outside
    const LeafGeometry* Get(int leaf) const
    {
        return mLeaves[leaf].get();
    }
    bool IsSolid(int leaf) const
    {
        return mState[leaf] == Solid;
    }

private:
    enum State : uint8_t
    {
        Missing,
        Empty,
        Solid,
        Surface
    };
    std::vector<std::unique_ptr<LeafGeometry>> mLeaves;
    std::vector<State> mState;
};

// The mesh of a stock, per mesh unit of its layout, updated when the stock changes. The surface
// of a changed leaf is replaced together with the meshes of all units that depend on it, so the
// mesh has no cracks at any time.
class DexelMeshUnits
{
public:
    // Start with a new stock, all units are built by the following updates
    void Reset(const DexelStock& stock);
    // Take the changed leaves of the stock and update the units depending on them, oldest changes
    // first, for about budgetMs. Returns the updated units.
    std::vector<int> Update(DexelStock& stock, ThreadPool& pool, double budgetMs);
    // Changed leaves are waiting
    bool Pending() const
    {
        return mFirst < mQueue.size();
    }
    const DexelMesh& Mesh(int unit) const
    {
        return mMeshes[unit];
    }
    // Updates the oldest leaf done by the last update waited for
    int LastDelay() const
    {
        return mLastDelay;
    }

private:
    std::shared_ptr<const DexelLayout> mLayout;
    DexelMeshCache mCache;
    std::vector<DexelMesh> mMeshes;
    std::vector<int> mQueue;  // changed leaves, from mFirst on
    std::vector<int> mQueuedAt;  // update of each queued leaf
    size_t mFirst = 0;
    int mUpdates = 0;
    int mLastDelay = 0;
    double mMsPerUnit = 0.05;  // measured time to update a unit
    std::vector<uint8_t> mQueued;
};

// Build the mesh of the given leaves of the stock layout, with dual contouring. The cache must
// hold the leaves and their neighbors, see DexelLayout::UnitDependencies.
//
// Grid points are inside the material if the Z dexel through them is. Cells of a brick are
// merged into larger cubes where the surface is simple. Every cube with a surface gets a vertex
// where the planes of the crossings of the dexels along its edges meet, so sharp edges and
// corners are kept. Every edge with a change of inside gets a quad of the vertices of the 4
// cubes around it. Edges are taken from the smallest cubes around them, where larger cubes make
// quads into triangles, so cubes of different sizes connect without cracks. Each edge belongs to
// one cube, so meshes of neighboring groups of leaves fit together.
void BuildDexelMesh(
    const DexelStock& stock,
    const DexelMeshCache& cache,
    const std::vector<int>& leaves,
    DexelMesh& mesh
);
// The same, computing the needed leaves
void BuildDexelMesh(const DexelStock& stock, const std::vector<int>& leaves, DexelMesh& mesh);

}  // namespace CAMSimulator
