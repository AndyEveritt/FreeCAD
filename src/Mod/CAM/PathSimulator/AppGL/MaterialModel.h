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

namespace CAMSimulator::Material
{

// Signed distance from the surface of the model of a job (a closed triangle mesh, counterclockwise
// seen from outside): positive outside the model, where material is left, negative inside, where
// the stock is cut into the model. Distances are measured up to a range, farther points get the
// range with their sign. Thread safe after construction.
class ModelDistance
{
public:
    ModelDistance(const std::vector<float>& xyz, const std::vector<uint32_t>& triangles, float range);

    float Range() const
    {
        return mRange;
    }
    bool Empty() const
    {
        return mTriangles.empty();
    }
    float SignedDistance(const float p[3]) const;

private:
    struct Triangle
    {
        float a[3], b[3], c[3];
        float n[3];  // unit normal
    };
    bool Inside(const float p[3]) const;
    int CellIndex(int x, int y, int z) const
    {
        return (z * mN[1] + y) * mN[0] + x;
    }

    float mRange;
    float mLo[3] = {0, 0, 0};
    float mCell = 1;
    int mN[3] = {1, 1, 1};
    std::vector<Triangle> mTriangles;
    // triangles by the cells their box (grown by the range) overlaps, CSR
    std::vector<uint32_t> mCellStart;
    std::vector<uint32_t> mCellTriangles;
    // triangles by the columns along Z their box overlaps, CSR, for the inside of far points
    std::vector<uint32_t> mColumnStart;
    std::vector<uint32_t> mColumnTriangles;
};

}  // namespace CAMSimulator::Material
