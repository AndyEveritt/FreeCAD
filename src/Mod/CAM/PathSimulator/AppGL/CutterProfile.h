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
    // The radius only
    float SliceRadius(float z) const;
    // Largest SliceRadius over the heights [z0, z1]
    float MaxSliceRadius(float z0, float z1, ProfileNormal& normal) const;
    // Radius of the part of the tool up to a height above its tip, at most Radius(): the
    // part that can cut when the material is lower than its shank
    float RadiusBelow(float height) const;
    // Smallest radius of curvature of the surface of the tool up to a height above its tip, 0 if
    // it has a sharp edge there. Moving the tool by d leaves ridges of at most d^2 / (8 * radius)
    // between two positions of it.
    float SmoothRadius(float height) const;

    // Heights of the vertices of the profile, sorted. SliceRadius is linear between them.
    const std::vector<float>& VertexHeights() const
    {
        return mVertexHeights;
    }

    // Size of the features the tool cuts: the radius of its cross section halfway up its lower
    // part, at most the tool radius
    float FeatureSize() const;

    // Lower edge of the cross section swept by the tool moving straight: the solid projected on
    // the plane perpendicular to the motion, its lowest points along 'up' by the position
    // 'across'. 'axis' is the tool axis in (along, across, up) coordinates, the tip is at the
    // origin. Sampled in 'bins' bins from -extent to extent, the edge of passes beside each
    // other is the same, so it gives the grooves between them for any tool shape and axis.
    struct SweptEdge
    {
        float start = 0;           // across of the first bin
        float step = 1;            // bin width
        std::vector<float> up;     // lowest point per bin, infinity where the tool isn't
        // lowest point at a position across, infinity outside
        float At(float across) const;
        // The grooves between passes beside each other at a spacing: the angle between the
        // surfaces the two passes leave at the ridge between them, and where the tool touches
        // the surface (across, up)
        void Groove(float spacing, float& angle, float& touchAcross, float& touchUp) const;
    };
    SweptEdge SweptLowerEdge(const float axis[3], float extent, int bins) const;

    // The solid as a stack of frustums between neighboring vertex heights: at a height z from z0
    // to z1 its cross section is the disk of radius r0 + (z - z0) * (r1 - r0) / (z1 - z0). Each
    // frustum is convex. Empty if the solid is not such a stack, like a tool with an overhang.
    struct Frustum
    {
        float z0, z1, r0, r1;
    };
    const std::vector<Frustum>& Frustums() const
    {
        return mFrustums;
    }

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
        ProfileNormal n1, n2;  // normals at the ends, between the edges at smooth corners
        bool sharp1, sharp2;   // the ends are sharp corners of the profile
    };

    // Normal of the tool surface at a point of an edge, interpolated between its ends
    static ProfileNormal NormalAt(const Edge& e, float rho, float z);

    std::vector<Edge> mEdges;
    std::vector<float> mVertexHeights;
    // edges reaching each range between vertex heights, from mSliceStart[i] to mSliceStart[i + 1]
    std::vector<int> mSliceStart;
    std::vector<int> mSliceEdges;
    // edges at the height z, the edges of its range
    template<class F>
    void ForEdgesAt(float z, F f) const;
    // smallest radius of curvature of every edge, 0 at sharp corners, from its lowest point on
    std::vector<std::pair<float, float>> mCurvature;
    std::vector<Frustum> mFrustums;
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
