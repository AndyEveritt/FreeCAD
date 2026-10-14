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

#include <vector>

namespace CAMSimulator
{

// A closed range along a line, e.g. of material or of a solid
struct Interval
{
    float lo;
    float hi;
};

// A rotationally symmetric cutter. The solid is the revolution around the tool axis of the region
// enclosed by the axis and the profile polyline. Coordinates are relative to the tool tip, with
// the axis pointing from the tip towards the shank.
class CutterProfile
{
public:
    // profile: radius/height pairs from the top of the tool down to the tip, as created by
    // CAMSimulation.GetToolProfile
    explicit CutterProfile(const std::vector<float>& profile);

    float Radius() const
    {
        return mRadius;
    }
    float Height() const
    {
        return mMaxZ;
    }

    // True if the solid at a distance from the axis contains the solid at every larger distance,
    // so a horizontal move removes the solid of its closest approach. True for all profiles with
    // one radius per height.
    bool IsRadiallyMonotonic() const
    {
        return mMonotonic;
    }

    // True if lines parallel to the axis enter and leave the solid at most once
    bool HasSingleAxialIntervals() const
    {
        return mSingleAxial;
    }

    bool Contains(float rho, float z) const;

    // Heights where a line parallel to the axis at distance d is inside the solid, looked up in a
    // precomputed table. d is rounded up to the next table entry. Returns the number of intervals
    // and points 'out' to them.
    int AxialIntervals(float d, const Interval*& out) const;

    // Exact version of AxialIntervals
    void ComputeAxialIntervals(float d, std::vector<Interval>& out) const;

    // Parameter ranges where the line origin + s * dir is inside the solid. Origin and direction
    // are given in tool coordinates, dir must be normalized. Writes the ranges in increasing
    // order to 'out'.
    void LineIntervals(const float origin[3], const float dir[3], std::vector<Interval>& out) const;

private:
    struct Edge
    {
        float r1, z1, r2, z2;
    };

    std::vector<Edge> mEdges;
    float mRadius = 0;
    float mMinZ = 0;
    float mMaxZ = 0;
    bool mMonotonic = true;
    bool mSingleAxial = true;

    // AxialIntervals table, entry i is for distance i * mTableStep
    float mTableStep = 1;
    std::vector<int> mTableStart;
    std::vector<Interval> mTableIntervals;
};

}  // namespace CAMSimulator
