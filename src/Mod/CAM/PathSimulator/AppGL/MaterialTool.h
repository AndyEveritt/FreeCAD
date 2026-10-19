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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace CAMSimulator::Material
{

// A rotationally symmetric cutter: the solid of revolution of its profile around the tool axis.
// Coordinates are relative to the tool tip, z along the axis from the tip towards the shank.
//
// The solid is decomposed into primitives whose sweeps along a straight move are convex: spheres
// (the arcs of the profile centered on the axis, like ball ends) and frustums (the straight edges,
// including cylinders and cones). The sweep of a convex primitive along a straight move with a
// fixed axis meets every line in one interval, found in closed form.
class Tool
{
public:
    // A sphere centered on the axis
    struct Sphere
    {
        double zc;
        double radius;
    };
    // A frustum between two heights, with the normals of its surface at both ends in the
    // (radius, height) half plane, interpolated between neighboring edges where the profile is
    // smooth, so faceted arcs (bull nose corners) cut smoothly shaded surfaces
    struct Frustum
    {
        double z0, z1;  // z0 < z1
        double r0, r1;
        double n0r, n0z;      // normal at z0
        double n1r, n1z;      // normal at z1
        bool sharp0, sharp1;  // the profile has a sharp corner at z0 / z1
    };

    // profile: radius/height pairs from the top of the tool down to its tip, the cross section of
    // the tool solid. cuts: false for tools that never remove material, like probes; profiles
    // without material above the tip are probes too.
    explicit Tool(const std::vector<float>& profile, bool cuts = true);

    bool Cuts() const
    {
        return mCuts;
    }
    const std::vector<Sphere>& Spheres() const
    {
        return mSpheres;
    }
    const std::vector<Frustum>& Frustums() const
    {
        return mFrustums;
    }
    double Radius() const
    {
        return mRadius;
    }
    double MinZ() const
    {
        return mMinZ;
    }
    double MaxZ() const
    {
        return mMaxZ;
    }
    // Largest radius of the solid at a height, negative outside its heights
    double RadiusAt(double z) const;
    // Lowest height of the solid at a distance from the axis, infinity beyond its radius
    double BottomAt(double rho) const;
    // Largest BottomAt over the distances up to rho: all material within rho of the axis above
    // it is inside the tool, for tools without undercuts
    double MaxBottom(double rho) const;
    // Largest radius of the solid from its lowest point up to a height above it
    double RadiusBelow(double height) const;
    // Size of the features the tool cuts: the radius of its cross section halfway up its lower
    // part, at most its radius. Small for the tips of engravers and V bits.
    double FeatureSize() const;
    // True if the point (radius, height) is inside the solid
    bool Contains(double r, double z) const;
    // True if every line along the axis meets the solid in one interval at most (no undercuts)
    bool SingleAxial() const
    {
        return mSingleAxial;
    }

private:
    void Build(std::vector<std::pair<double, double>> points);

    std::vector<std::pair<double, double>> mPolygon;  // closed, (r, z)
    std::vector<Sphere> mSpheres;
    std::vector<Frustum> mFrustums;
    double mRadius = 0;
    double mMinZ = 0;
    double mMaxZ = 0;
    bool mCuts = true;
    bool mSingleAxial = true;
};

// Packed unit normal of a material surface (octahedral, 20 bits: within about 0.1 degrees), the
// tag of the operation that made it (10 bits), and flags: made by cutting, normal unknown (made by
// a sharp edge of a tool).
constexpr uint32_t NoNormal = 1u << 30;
constexpr uint32_t CutFlag = 1u << 31;
constexpr int TagShift = 20;
constexpr uint32_t TagMask = 1023u << TagShift;
// operations get the tags 1 to MaxTag, more operations share them
constexpr int MaxTag = 1023;
inline int OperationTag(int operation)
{
    return operation < 0 ? 0 : operation % MaxTag + 1;
}
uint32_t PackNormal(const float n[3]);
void UnpackNormal(uint32_t packed, float n[3]);
inline bool HasNormal(uint32_t packed)
{
    return (packed & NoNormal) == 0;
}
inline bool IsCut(uint32_t packed)
{
    return (packed & CutFlag) != 0;
}
inline int TagOf(uint32_t packed)
{
    return (int)((packed & TagMask) >> TagShift);
}

// A straight motion of a tool: the tip moves linearly from p0 to p1 while the axis (unit vector
// from the tip towards the shank) turns from a0 to a1 (normalized linear interpolation).
struct Motion
{
    const Tool* tool = nullptr;
    double p0[3] = {};
    double p1[3] = {};
    double a0[3] = {0, 0, 1};
    double a1[3] = {0, 0, 1};
    int tag = 0;  // operation tag of the faces it cuts
    // Height above the tip beyond which the tool can't reach the material, like the shank above
    // the stock: the parts of the tool above it are left out
    double cutHeight = 1e30;
    // Largest deviation of the swept volume of a motion turning the axis
    double tolerance = 5e-4;
    // A circular arc instead of a line, for tools along Z at a constant height: the tip is at
    // center + radius (-sin(a), cos(a)), a = startAngle + t sweep, |sweep| at most pi / 2, and at
    // the height of p0
    bool arc = false;
    double center[2] = {0, 0};
    double radius = 0;
    double startAngle = 0;
    double sweep = 0;
};

// Distance in the plane from a point to the arc of a motion, and the time of its closest point
double ArcDistance(const Motion& m, double x, double y, double& t);

// The arc of a motion, for fast distances: the directions of its ends from its center
struct ArcGeometry
{
    double cx = 0, cy = 0, radius = 0;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;      // ends
    double d0x = 0, d0y = 0, d1x = 0, d1y = 0;  // directions of the ends from the center
    double side = 1;                            // 1 for counterclockwise
    explicit ArcGeometry(const Motion& m);
    ArcGeometry() = default;
    // distance in the plane from a point to the arc
    double Distance(double x, double y) const
    {
        const double dx = x - cx, dy = y - cy;
        // in the wedge of the arc (up to a half circle): on the inner side of both ends
        if (side * (d0x * dy - d0y * dx) >= 0 && side * (dx * d1y - dy * d1x) >= 0) {
            return std::abs(std::sqrt(dx * dx + dy * dy) - radius);
        }
        const double e0 = (x - x0) * (x - x0) + (y - y0) * (y - y0);
        const double e1 = (x - x1) * (x - x1) + (y - y1) * (y - y1);
        return std::sqrt(std::min(e0, e1));
    }
};

// Distance from a point to the parallelogram base + x e1 + y e2, x and y in [0, 1], in 3D, or in
// 2D across a world axis (the components along 'axis' are ignored) if axis >= 0
double ParallelogramDistance(
    const double p[3],
    const double base[3],
    const double e1[3],
    const double e2[3],
    int axis = -1
);

// Height above the tip of the parts of a tool that can reach into a box during a motion, an upper
// bound. Below the lowest point of the tool if it stays out of the box.
double CuttingHeight(const Motion& motion, const double lo[3], const double hi[3]);

// An end of a removal along a line: the parameter along the line and how to find the normal of
// the surface there
struct RemovalEnd
{
    double s;
    int piece;      // sweep piece of the motion
    int primitive;  // index: spheres first, then frustums
    int kind;       // see Sweep::EndKind
    double t;       // time of the motion piece when the tool touches the end
};

struct Removal
{
    RemovalEnd lo;
    RemovalEnd hi;
};

// The volume swept by a tool along a motion, intersected with axis aligned lines.
//
// Motions with a fixed axis are swept exactly: the sweep of every primitive along a line is the
// projection of a convex region of (line parameter, time), whose extremes are found in closed
// form. Motions that turn the axis are divided into pieces that turn by a small angle, each swept
// with the axis fixed at its middle on a slightly enlarged tool, then the ends are refined on the
// exact turning tool, so turning tools are as accurate as fixed ones.
class Sweep
{
public:
    enum EndKind
    {
        SphereSurface,
        ConeSurface,
        CapBottom,
        CapTop,
        Rim0,  // the rim of a frustum at z0
        Rim1,
    };

    Sweep() = default;
    explicit Sweep(const Motion& motion);

    const Motion& GetMotion() const
    {
        return mMotion;
    }
    // Bounding box of the swept volume
    const double* BoxLo() const
    {
        return mLo;
    }
    const double* BoxHi() const
    {
        return mHi;
    }
    // False if no point within 'margin' of 'p' is in the swept volume (motions with a fixed axis:
    // the swept cylinder around the tool, true for turning motions)
    bool MayTouch(const double p[3], double margin) const;

    // Material removed along the line through 'origin' along the world axis 'axis', for
    // parameters s in [sMin, sMax] (world coordinate along the axis). Appends the removed
    // intervals, sorted and disjoint, and returns how many.
    int Line(int axis, const double origin[3], double sMin, double sMax, std::vector<Removal>& out) const;

    // Normal of the material surface at an end (pointing out of the material), unit length, or
    // false if unknown (sharp edges of the tool)
    bool Normal(int axis, const double origin[3], const RemovalEnd& end, float n[3]) const;

private:
    // A primitive of the tool, frustums cut at the cutting height
    struct Prim
    {
        bool sphere;
        int index;              // in the spheres or frustums of the tool
        int index1;             // frustum at the top of merged frustums
        double z0, z1, r0, r1;  // frustum
        double zc, radius;      // sphere
        double zm, bound;       // the primitive is inside the sphere of this radius around zm
    };
    struct Piece
    {
        double p0[3];
        double d[3];                // p1 - p0
        double e1[3], e2[3], a[3];  // tool frame
    };
    // where a primitive sweeps in a piece: its box, and the segment its center sweeps
    struct Bound
    {
        float lo[3], hi[3];
        double c0[3], c1[3];
    };
    // bounding volume hierarchy of the pieces of turning motions
    struct Node
    {
        float lo[3], hi[3];
        int first, count;  // pieces, or children if count < 0: first and right
        int right;
        // the pieces of one primitive: their sweep is inside the sweep of the primitive grown
        // by 'grow' with a fixed axis over their times
        int prim = -1;
        double grow = 0;
        Piece pose;
    };
    struct Interval
    {
        RemovalEnd lo, hi;
    };
    void LinePiece(
        int piece,
        int axis,
        const double o[3],
        double sMin,
        double sMax,
        std::vector<Interval>& found
    ) const;
    // a piece of a primitive of a turning motion
    void LineEntry(
        int entry,
        int axis,
        const double o[3],
        double sMin,
        double sMax,
        std::vector<Interval>& found
    ) const;
    static Bound BoundOf(const Piece& p, const Prim& prim);
    // the piece of a primitive at its center from t0 to t1, with the axis of the middle
    Piece PieceOf(const Prim& prim, double t0, double t1) const;
    bool SphereLine(const Piece& p, const Prim& prim, int axis, const double o[3], Interval& out) const;
    bool FrustumLine(const Piece& p, const Prim& prim, int axis, const double o[3], Interval& out) const;
    void PoseAt(double t, double tip[3], double axisDir[3]) const;
    int BuildNode(int first, int count);
    // the disk of a cross section swept along the arc, on a line across the axis
    void ArcDiskLine(int axis, const double o[3], double radius, int prim, std::vector<Interval>& found) const;
    static double PrimLow(const Prim& p);
    static double PrimHigh(const Prim& p);
    static double PrimRadius(const Prim& p);

    Motion mMotion;
    std::vector<Prim> mPrims;
    // the primitives by increasing lowest height, with the prefix maxima of their radii and of
    // their highest heights, and by decreasing highest height with the prefix maxima of radii
    std::vector<int> mByLow;
    std::vector<double> mByLowRadius;
    std::vector<double> mByLowTop;
    std::vector<int> mByHigh;
    std::vector<double> mByHighRadius;
    // fixed axis: one piece for all primitives, bounds by primitive; turning: the pieces of all
    // primitives, their primitives and bounds
    bool mTurning = false;
    std::vector<Piece> mPieces;
    std::vector<int> mEntryPrim;
    std::vector<double> mEntryT0, mEntryT1;  // times of the pieces
    std::vector<Bound> mBounds;
    std::vector<Node> mNodes;
    double mLo[3] = {};
    double mHi[3] = {};
    ArcGeometry mArc;
    double mSlabBase[3] = {};
    double mSlabE1[3] = {};
    double mSlabE2[3] = {};
    double mSlabRadius = -1;
};

}  // namespace CAMSimulator::Material
