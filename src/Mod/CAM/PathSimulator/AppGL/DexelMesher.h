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
#include <vector>

namespace CAMSimulator
{

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

// Build the mesh of the cells [ix0, ix1) x [iy0, iy1) of the stock. Faces are cut faces where they
// are not on the surface of the initial stock.
//
// Tops of neighboring cells within a few cells of height of each other form one smooth surface:
// the corners of the cell tops are at the average height of the surface tops around them, so the
// tops connect without side faces. Side faces are only made between cells not on one surface.
// Equal flat faces are merged into rectangles.
void BuildDexelMesh(
    const DexelStock& stock,
    const DexelStock& initial,
    int ix0,
    int iy0,
    int ix1,
    int iy1,
    DexelMesh& mesh
);

}  // namespace CAMSimulator
