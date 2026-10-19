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

#include "MaterialGrid.h"
#include "MaterialModel.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace CAMSimulator
{
class ThreadPool;
}

namespace CAMSimulator::Material
{

// Material is measured up to this thickness
constexpr float MaxThickness = 10;
// Thickness of vertices that were not measured, or thicker than measured
constexpr float NoThickness = 1e30f;
// Thickness as a fraction of MaxThickness in 16 bits (steps of 0.15 um), the largest value for
// none
constexpr uint16_t NoThicknessCode = 65535;
inline uint16_t EncodeThickness(float t)
{
    return t >= MaxThickness ? NoThicknessCode : (uint16_t)(t / MaxThickness * 65534 + 0.5f);
}
inline float DecodeThickness(uint16_t code)
{
    return code == NoThicknessCode ? NoThickness : code * (MaxThickness / 65534);
}

// Distance of vertices from the model that was not measured
constexpr float NoDeviation = 1e30f;

// Vertex of the stock mesh: position, normal, the tag of the operation that cut its face (0 for
// none), the thickness of the material behind it (EncodeThickness) and its signed distance from
// the model (see ModelDistance)
struct MeshVertex
{
    float x, y, z;
    int8_t nx, ny, nz;
    uint8_t reserved;
    uint16_t tag;
    uint16_t thickness;
    float deviation;
};
static_assert(sizeof(MeshVertex) == 24);

// Triangles of a mesh unit: faces of the original stock and cut faces
struct UnitMesh
{
    std::vector<MeshVertex> vertices;
    std::vector<uint32_t> stock;
    std::vector<uint32_t> cut;
};

// The surface of the stock as a mesh, with dual contouring of the tri-dexels.
//
// Grid points are inside the material if the dexel along Z through them is. The edges between grid
// points lie on dexels, so where the inside changes along an edge, the dexel has the exact point of
// the surface and its normal. The cells of a brick are merged into cubes of 2, 4 or 8 cells where
// the surface in them is flat: its normals agree within a few degrees and its points are within a
// tenth of a cell of a plane, so flat faces get few large triangles. Every cube with a surface gets
// a vertex where the planes of the points on its edges meet, so sharp edges and corners are kept.
// Every edge of the cubes with a change of inside gets a quad of the vertices of the 4 cubes around
// it, from the smallest cubes around it, so bricks of different levels connect without cracks.
// Vertices on smooth surfaces share the normal of their surface points, at sharp edges every quad
// gets the normal of its own point.
//
// The mesh is built per mesh unit of the layout and rebuilt when the leaves it depends on change.
class Mesher
{
public:
    Mesher();
    ~Mesher();

    // Start over with a layout, all units are built by the next update
    void Reset(std::shared_ptr<const Layout> layout, ThreadPool& pool);
    // Measure the thickness of the material behind the surface; a change rebuilds all units
    void SetMeasureThickness(bool b);
    bool MeasuresThickness() const
    {
        return mThickness;
    }
    // Measure the distance of the vertices from the model, if any; a change rebuilds all units
    void SetDeviation(std::shared_ptr<const ModelDistance> model);
    const std::shared_ptr<const ModelDistance>& Deviation() const
    {
        return mModel;
    }
    // Update the meshes depending on the changed leaves of the grid, returns the rebuilt units
    std::vector<int> Update(const Grid& grid, const std::vector<int>& changed, ThreadPool& pool);
    // All units are rebuilt by the next update
    void InvalidateAll();
    const UnitMesh& Mesh(int unit) const
    {
        return mMeshes[unit];
    }
    int UnitCount() const
    {
        return (int)mMeshes.size();
    }
    const Layout& GetLayout() const
    {
        return *mLayout;
    }
    // Memory of the cache and the meshes
    long MemoryBytes() const;

    struct Geometry;

private:
    void ComputeGeometry(const Grid& grid, int leaf);
    void BuildUnit(const Grid& grid, int unit, UnitMesh& out) const;

    std::shared_ptr<const Layout> mLayout;
    // leaves touching each leaf, CSR
    std::vector<int> mNeighborStart;
    std::vector<int> mNeighbors;
    std::vector<std::unique_ptr<Geometry>> mGeometry;
    std::vector<UnitMesh> mMeshes;
    bool mThickness = false;
    std::shared_ptr<const ModelDistance> mModel;
    bool mAll = true;
    bool mAllUnits = false;  // all units, from the geometry kept
};

}  // namespace CAMSimulator::Material
