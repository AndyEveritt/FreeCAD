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

// Normal of a tool surface in the (radius, height) half plane of the profile, pointing out of the
// solid
struct ProfileNormal
{
    float r;
    float z;
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
    // so a horizontal move removes the solid of its closest approach, and every cross section
    // perpendicular to the axis is a disk. True for all profiles with one radius per height.
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
    // and points 'out' to them, and 'normals' to the normals at their ends (lo, hi).
    int AxialIntervals(float d, const Interval*& out, const ProfileNormal*& normals) const;
    int AxialIntervals(float d, const Interval*& out) const
    {
        const ProfileNormal* normals;
        return AxialIntervals(d, out, normals);
    }

    // Exact version of AxialIntervals
    void ComputeAxialIntervals(
        float d,
        std::vector<Interval>& out,
        std::vector<ProfileNormal>* normals = nullptr
    ) const;

    // Radius of the cross section at height z, the largest distance from the axis inside the
    // solid, and the normal of the surface there. Returns a negative radius if the solid doesn't
    // reach that height.
    float SliceRadius(float z, ProfileNormal& normal) const;
    // Largest SliceRadius over the heights [z0, z1]
    float MaxSliceRadius(float z0, float z1, ProfileNormal& normal) const;

    // Heights of the vertices of the profile, sorted. SliceRadius is linear between them.
    const std::vector<float>& VertexHeights() const
    {
        return mVertexHeights;
    }

    // Size of the features the tool cuts: the radius of its cross section halfway up its lower
    // part, at most the tool radius
    float FeatureSize() const;

    // Parameter ranges where the line origin + s * dir is inside the solid. Origin and direction
    // are given in tool coordinates, dir must be normalized. Writes the ranges in increasing
    // order to 'out', and if given the surface normals at their ends (tool coordinates, out of
    // the solid) to 'normals', 6 floats per range. Within 'sharpTolerance' of a sharp edge of the
    // tool, the normal of a surface it cuts is not known: the normal is zero there.
    void LineIntervals(
        const float origin[3],
        const float dir[3],
        std::vector<Interval>& out,
        std::vector<float>* normals = nullptr,
        float sharpTolerance = 0
    ) const;

private:
    struct Edge
    {
        float r1, z1, r2, z2;
        ProfileNormal normal;
        bool sharp1, sharp2;  // the ends are sharp corners of the profile
    };

    std::vector<Edge> mEdges;
    std::vector<float> mVertexHeights;
    float mRadius = 0;
    float mMinZ = 0;
    float mMaxZ = 0;
    bool mMonotonic = true;
    bool mSingleAxial = true;
    bool mConvex = true;

    // AxialIntervals table, entry i is for distance i * mTableStep
    float mTableStep = 1;
    std::vector<int> mTableStart;
    std::vector<Interval> mTableIntervals;
    std::vector<ProfileNormal> mTableNormals;  // 2 per interval
};

}  // namespace CAMSimulator
