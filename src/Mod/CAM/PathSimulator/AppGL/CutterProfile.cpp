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

#include "CutterProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CAMSimulator
{

constexpr int TableSize = 1024;

CutterProfile::CutterProfile(const std::vector<float>& profile)
{
    // closed polygon in the (radius, height) half plane: from the axis at the top, along the
    // profile, back to the axis at the bottom
    std::vector<std::pair<float, float>> points;
    for (size_t i = 0; i + 1 < profile.size(); i += 2) {
        points.emplace_back(std::max(0.0f, profile[i]), profile[i + 1]);
    }
    if (points.empty()) {
        return;
    }
    if (points.front().first > 0) {
        points.insert(points.begin(), {0.0f, points.front().second});
    }
    if (points.back().first > 0) {
        points.emplace_back(0.0f, points.back().second);
    }

    // orientation of the polygon, for the outward normals
    double area = 0;
    for (size_t i = 0; i < points.size(); i++) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % points.size()];
        area += (double)a.first * b.second - (double)b.first * a.second;
    }
    const float side = area > 0 ? 1.0f : -1.0f;

    // outward normals of the polygon edges, and the corners where they turn sharply
    const size_t n = points.size();
    std::vector<ProfileNormal> edgeNormals(n, ProfileNormal {0, 0});
    for (size_t i = 0; i < n; i++) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % n];
        const float dr = b.first - a.first, dz = b.second - a.second;
        const float len = std::hypot(dr, dz);
        if (len > 0) {
            // right of the edge for a counter clockwise polygon
            edgeNormals[i] = {side * dz / len, -side * dr / len};
        }
    }
    // convex if the polygon turns the same way at every corner
    for (size_t i = 0; i < n && mConvex; i++) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % n];
        const auto& c = points[(i + 2) % n];
        const double cross = (double)(b.first - a.first) * (c.second - b.second)
            - (double)(b.second - a.second) * (c.first - b.first);
        mConvex = cross * side >= -1e-9;
    }
    auto onAxis = [&](size_t i) {
        return points[i].first == 0 && points[(i + 1) % n].first == 0;
    };
    std::vector<bool> sharp(n, false);
    for (size_t i = 0; i < n; i++) {
        const size_t before = (i + n - 1) % n;
        if (onAxis(before) || onAxis(i)) {
            // on the axis: the edge meets its mirror image, the tip of a cone is sharp, the
            // middle of a flat face or the tip of a ball is not
            const ProfileNormal& m = onAxis(before) ? edgeNormals[i] : edgeNormals[before];
            sharp[i] = 1 - 2 * m.r * m.r < 0.94f;
        }
        else {
            sharp[i] = edgeNormals[before].r * edgeNormals[i].r + edgeNormals[before].z * edgeNormals[i].z < 0.94f;
        }
    }

    // normals at the vertices: between the normals of the edges where the profile is smooth, so
    // the normals of a curved profile approximated by edges change continuously like on the tool
    std::vector<ProfileNormal> vertexNormals(n);
    for (size_t i = 0; i < n; i++) {
        const size_t before = (i + n - 1) % n;
        ProfileNormal m {0, 0};
        if (onAxis(before) || onAxis(i)) {
            const ProfileNormal& e = onAxis(before) ? edgeNormals[i] : edgeNormals[before];
            m = {0, e.z < 0 ? -1.0f : 1.0f};
        }
        else {
            m = {edgeNormals[before].r + edgeNormals[i].r, edgeNormals[before].z + edgeNormals[i].z};
            const float len = std::hypot(m.r, m.z);
            m = len > 0 ? ProfileNormal {m.r / len, m.z / len} : edgeNormals[i];
        }
        vertexNormals[i] = m;
    }

    mMinZ = mMaxZ = points.front().second;
    for (size_t i = 0; i < n; i++) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % n];
        mRadius = std::max(mRadius, a.first);
        mMinZ = std::min(mMinZ, a.second);
        mMaxZ = std::max(mMaxZ, a.second);
        if (onAxis(i) || (a.first == b.first && a.second == b.second)) {
            continue;
        }
        const size_t next = (i + 1) % n;
        mEdges.push_back({
            a.first,
            a.second,
            b.first,
            b.second,
            edgeNormals[i],
            sharp[i] ? edgeNormals[i] : vertexNormals[i],
            sharp[next] ? edgeNormals[i] : vertexNormals[next],
            sharp[i],
            sharp[next],
        });
    }

    for (const auto& p : points) {
        mVertexHeights.push_back(p.second);
    }
    std::sort(mVertexHeights.begin(), mVertexHeights.end());
    mVertexHeights.erase(std::unique(mVertexHeights.begin(), mVertexHeights.end()), mVertexHeights.end());
    // curvature of the edges: along the profile from the turn of the normals between the ends,
    // around the axis from the distance to the axis along the normal. Sharp corners at their
    // height.
    for (const Edge& e : mEdges) {
        if (e.sharp1) {
            mCurvature.emplace_back(e.z1, 0.0f);
        }
        if (e.sharp2) {
            mCurvature.emplace_back(e.z2, 0.0f);
        }
        float radius = std::numeric_limits<float>::infinity();
        const float length = std::hypot(e.r2 - e.r1, e.z2 - e.z1);
        if (length <= 1e-6f * mRadius) {
            continue;  // a point
        }
        const float turn = std::acos(std::clamp(e.n1.r * e.n2.r + e.n1.z * e.n2.z, -1.0f, 1.0f));
        if (turn > 1e-6f) {
            radius = length / turn;
        }
        for (const auto& [r, n] : {std::make_pair(e.r1, e.n1.r), std::make_pair(e.r2, e.n2.r)}) {
            if (r > 1e-6f && std::abs(n) > 1e-6f) {
                radius = std::min(radius, r / std::abs(n));
            }
        }
        mCurvature.emplace_back(std::min(e.z1, e.z2), radius);
    }
    // the edges reaching the range between neighboring vertex heights, including its ends
    for (size_t i = 0; i + 1 < std::max<size_t>(mVertexHeights.size(), 2); i++) {
        mSliceStart.push_back((int)mSliceEdges.size());
        if (mVertexHeights.size() < 2) {
            break;
        }
        const float lo = mVertexHeights[i], hi = mVertexHeights[i + 1];
        for (int e = 0; e < (int)mEdges.size(); e++) {
            if (std::min(mEdges[e].z1, mEdges[e].z2) <= hi && std::max(mEdges[e].z1, mEdges[e].z2) >= lo) {
                mSliceEdges.push_back(e);
            }
        }
    }
    mSliceStart.push_back((int)mSliceEdges.size());


    // table of axial intervals
    mTableStep = mRadius > 0 ? mRadius / (TableSize - 1) : 1;
    std::vector<Interval> intervals;
    std::vector<ProfileNormal> normals;
    mTableStart.reserve(TableSize + 1);
    for (int i = 0; i < TableSize; i++) {
        mTableStart.push_back((int)mTableIntervals.size());
        ComputeAxialIntervals(i * mTableStep, intervals, &normals);
        mTableIntervals.insert(mTableIntervals.end(), intervals.begin(), intervals.end());
        mTableNormals.insert(mTableNormals.end(), normals.begin(), normals.end());
    }
    mTableStart.push_back((int)mTableIntervals.size());

    for (int i = 0; i < TableSize; i++) {
        mSingleAxial &= mTableStart[i + 1] - mTableStart[i] <= 1;
    }

    // every entry must be contained in the previous one, up to a hundredth of the radius: tools
    // often have tiny steps where the flutes meet the shank, which don't change the cut
    const float slack = std::max(1e-5f, 0.01f * mRadius);
    for (int i = 1; i < TableSize && mMonotonic; i++) {
        for (int k = mTableStart[i]; k < mTableStart[i + 1]; k++) {
            const Interval& inner = mTableIntervals[k];
            bool contained = false;
            for (int j = mTableStart[i - 1]; j < mTableStart[i]; j++) {
                const Interval& outer = mTableIntervals[j];
                contained |= outer.lo <= inner.lo + slack && inner.hi <= outer.hi + slack;
            }
            mMonotonic &= contained;
        }
    }

    // frustums: between neighboring vertex heights, the solid from the axis to the one edge
    // crossing the range. Or to the outermost edge in ranges as thin as the tiny steps where the
    // flutes meet the shank, see above.
    for (size_t i = 0; i + 1 < mVertexHeights.size(); i++) {
        const float lo = mVertexHeights[i], hi = mVertexHeights[i + 1];
        const float mid = (lo + hi) / 2;
        float r0 = -1, r1 = -1;
        int count = 0;
        for (int k = mSliceStart[i]; k < mSliceStart[i + 1]; k++) {
            const Edge& e = mEdges[mSliceEdges[k]];
            if (std::min(e.z1, e.z2) < mid && std::max(e.z1, e.z2) > mid) {
                r0 = std::max(r0, e.r1 + (lo - e.z1) * (e.r2 - e.r1) / (e.z2 - e.z1));
                r1 = std::max(r1, e.r1 + (hi - e.z1) * (e.r2 - e.r1) / (e.z2 - e.z1));
                count++;
            }
        }
        if (count == 0 || (count > 1 && hi - lo > slack)) {
            mFrustums.clear();
            break;
        }
        mFrustums.push_back({lo, hi, std::max(0.0f, r0), std::max(0.0f, r1)});
    }
}

ProfileNormal CutterProfile::NormalAt(const Edge& e, float rho, float z)
{
    const float dr = e.r2 - e.r1, dz = e.z2 - e.z1;
    const float len2 = dr * dr + dz * dz;
    const float t = len2 > 0 ? std::clamp(((rho - e.r1) * dr + (z - e.z1) * dz) / len2, 0.0f, 1.0f) : 0.0f;
    const ProfileNormal n {e.n1.r + t * (e.n2.r - e.n1.r), e.n1.z + t * (e.n2.z - e.n1.z)};
    const float len = std::hypot(n.r, n.z);
    return len > 0 ? ProfileNormal {n.r / len, n.z / len} : e.normal;
}

bool CutterProfile::Contains(float rho, float z) const
{
    // crossings of a ray from the point towards larger heights
    bool inside = false;
    for (const Edge& e : mEdges) {
        if ((e.r1 <= rho) != (e.r2 <= rho)) {
            const float zc = e.z1 + (rho - e.r1) * (e.z2 - e.z1) / (e.r2 - e.r1);
            if (zc > z) {
                inside = !inside;
            }
        }
    }
    return inside;
}

void CutterProfile::ComputeAxialIntervals(
    float d,
    std::vector<Interval>& out,
    std::vector<ProfileNormal>* normals
) const
{
    out.clear();
    if (normals) {
        normals->clear();
    }
    std::vector<std::pair<float, int>> crossings;
    for (int i = 0; i < (int)mEdges.size(); i++) {
        const Edge& e = mEdges[i];
        // half open, so a vertex is counted once
        if ((e.r1 <= d) != (e.r2 <= d)) {
            crossings.emplace_back(e.z1 + (d - e.r1) * (e.z2 - e.z1) / (e.r2 - e.r1), i);
        }
    }
    std::sort(crossings.begin(), crossings.end());
    for (size_t i = 0; i + 1 < crossings.size(); i += 2) {
        if (crossings[i + 1].first > crossings[i].first) {
            out.push_back({crossings[i].first, crossings[i + 1].first});
            if (normals) {
                normals->push_back(NormalAt(mEdges[crossings[i].second], d, crossings[i].first));
                normals->push_back(NormalAt(mEdges[crossings[i + 1].second], d, crossings[i + 1].first));
            }
        }
    }
}

int CutterProfile::AxialIntervals(float d, const Interval*& out, const ProfileNormal*& normals) const
{
    const int index = (int)std::ceil(d / mTableStep);
    if (index < 0 || index >= TableSize) {
        out = nullptr;
        normals = nullptr;
        return 0;
    }
    out = mTableIntervals.data() + mTableStart[index];
    normals = mTableNormals.data() + 2 * mTableStart[index];
    return mTableStart[index + 1] - mTableStart[index];
}

template<class F>
void CutterProfile::ForEdgesAt(float z, F f) const
{
    if (mVertexHeights.size() < 2 || z < mVertexHeights.front() || z > mVertexHeights.back()) {
        return;
    }
    const int range = std::clamp(
        (int)(std::upper_bound(mVertexHeights.begin(), mVertexHeights.end(), z) - mVertexHeights.begin()) - 1,
        0,
        (int)mVertexHeights.size() - 2
    );
    for (int i = mSliceStart[range]; i < mSliceStart[range + 1]; i++) {
        f(mEdges[mSliceEdges[i]]);
    }
}

float CutterProfile::SliceRadius(float z) const
{
    float best = -1;
    ForEdgesAt(z, [&](const Edge& e) {
        const float zlo = std::min(e.z1, e.z2), zhi = std::max(e.z1, e.z2);
        if (z < zlo || z > zhi) {
            return;
        }
        const float r = e.z1 == e.z2 ? std::max(e.r1, e.r2)
                                     : e.r1 + (z - e.z1) * (e.r2 - e.r1) / (e.z2 - e.z1);
        best = std::max(best, r);
    });
    return best;
}

float CutterProfile::SliceRadius(float z, ProfileNormal& normal) const
{
    float best = -1;
    const Edge* bestEdge = nullptr;
    ForEdgesAt(z, [&](const Edge& e) {
        const float zlo = std::min(e.z1, e.z2), zhi = std::max(e.z1, e.z2);
        if (z < zlo || z > zhi) {
            return;
        }
        const float r = e.z1 == e.z2 ? std::max(e.r1, e.r2)
                                     : e.r1 + (z - e.z1) * (e.r2 - e.r1) / (e.z2 - e.z1);
        // at a vertex shared by two edges, prefer the edge facing outwards
        if (r > best || (r == best && bestEdge && e.normal.r > bestEdge->normal.r)) {
            best = r;
            bestEdge = &e;
        }
    });
    if (bestEdge) {
        normal = NormalAt(*bestEdge, best, z);
    }
    return best;
}

float CutterProfile::MaxSliceRadius(float z0, float z1, ProfileNormal& normal) const
{
    float best = SliceRadius(z0, normal);
    ProfileNormal n;
    const float r1 = SliceRadius(z1, n);
    if (r1 > best) {
        best = r1;
        normal = n;
    }
    // the largest radius is at an end of the range or at a profile vertex inside it
    for (const Edge& e : mEdges) {
        for (const float z : {e.z1, e.z2}) {
            if (z > z0 && z < z1) {
                const float r = SliceRadius(z, n);
                if (r > best) {
                    best = r;
                    normal = n;
                }
            }
        }
    }
    return best;
}

float CutterProfile::RadiusBelow(float height) const
{
    if (height >= mMaxZ) {
        return mRadius;
    }
    ProfileNormal unused;
    return std::clamp(MaxSliceRadius(mMinZ, std::max(mMinZ, height), unused), 0.0f, mRadius);
}

float CutterProfile::SmoothRadius(float height) const
{
    float radius = std::numeric_limits<float>::infinity();
    for (const auto& [z, r] : mCurvature) {
        if (z < height) {
            radius = std::min(radius, r);
        }
    }
    return std::isfinite(radius) ? radius : 0.0f;
}

float CutterProfile::FeatureSize() const
{
    ProfileNormal normal;
    const float r = SliceRadius(std::min(mRadius, mMaxZ - mMinZ) / 2 + mMinZ, normal);
    return std::clamp(r, mRadius / 4, mRadius);
}

float CutterProfile::SweptEdge::At(float across) const
{
    const float x = (across - start) / step;
    const int i = (int)std::floor(x);
    if (i < 0 || i + 1 >= (int)up.size()) {
        return std::numeric_limits<float>::infinity();
    }
    const float f = x - i;
    if (!std::isfinite(up[i]) || !std::isfinite(up[i + 1])) {
        return f < 0.5f ? up[i] : up[i + 1];
    }
    return up[i] + f * (up[i + 1] - up[i]);
}

void CutterProfile::SweptEdge::Groove(float spacing, float& angle, float& touchAcross, float& touchUp) const
{
    const int bins = (int)up.size();
    float lowest = std::numeric_limits<float>::infinity();
    for (const float v : up) {
        lowest = std::min(lowest, v);
    }
    // the middle of the lowest points, a flat bottom touches with all of them
    const float flat = lowest + 1e-3f * step;
    int first = -1, last = -1;
    for (int b = 0; b < bins; b++) {
        if (up[b] <= flat) {
            last = b;
            if (first < 0) {
                first = b;
            }
        }
    }
    touchAcross = start + (first + last) / 2.0f * step;
    touchUp = lowest;
    // the ridge: the passes at 0 and spacing leave the same height between their lowest points,
    // in the middle where they overlap with flat bottoms
    const float same = 1e-3f * step;
    float from = touchAcross + spacing, to = touchAcross;
    for (float u = touchAcross; u <= touchAcross + spacing; u += step / 2) {
        const float difference = At(u) - At(u - spacing);
        if (difference >= -same) {
            from = std::min(from, u);
        }
        if (difference <= same) {
            to = std::max(to, u);
        }
    }
    const float ridge = from <= to ? (from + to) / 2 : from;
    // slopes of the two surfaces at the ridge, over a tenth of the spacing
    angle = 0;
    const float d = std::max(4 * step, 0.05f * spacing);
    const float a1 = At(ridge + d), a0 = At(ridge - d);
    const float b1 = At(ridge - spacing + d), b0 = At(ridge - spacing - d);
    if (std::isfinite(a0 + a1 + b0 + b1)) {
        angle = std::atan((a1 - a0) / (2 * d)) - std::atan((b1 - b0) / (2 * d));
    }
}

CutterProfile::SweptEdge CutterProfile::SweptLowerEdge(const float axis[3], float extent, int bins) const
{
    SweptEdge edge;
    edge.start = -extent;
    edge.step = 2 * extent / (bins - 1);
    edge.up.assign(bins, std::numeric_limits<float>::infinity());
    // directions in the plane of a slice of the tool, perpendicular to the axis
    const double a[3] = {axis[0], axis[1], axis[2]};
    double p[3] = {0, 1, 0};  // across, made perpendicular to the axis
    if (std::abs(a[1]) > 0.9) {
        p[0] = 1;
        p[1] = 0;
    }
    const double pa = p[0] * a[0] + p[1] * a[1] + p[2] * a[2];
    double len = 0;
    for (int k = 0; k < 3; k++) {
        p[k] -= pa * a[k];
        len += p[k] * p[k];
    }
    len = std::sqrt(len);
    for (double& c : p) {
        c /= len;
    }
    const double q[3] = {a[1] * p[2] - a[2] * p[1], a[2] * p[0] - a[0] * p[2], a[0] * p[1] - a[1] * p[0]};

    // circles along the edges of the profile, apart by less than half a bin. Where the profile is
    // smooth, the edges are bent to the normals at their ends, like the arcs they approximate, so
    // the edge follows the shape of the tool and not its polygon.
    struct Slice
    {
        float z, radius, lowest;
    };
    std::vector<Slice> slices;
    const float top = std::min(mMaxZ, mMinZ + std::max(2.0f * extent, mRadius));
    // the lowest point of the projected outline of a slice
    const double spread = std::sqrt(p[2] * p[2] + q[2] * q[2]);
    float lowestAll = std::numeric_limits<float>::infinity();
    for (const Edge& e : mEdges) {
        if (std::min(e.z1, e.z2) > top) {
            continue;
        }
        const float dr = e.r2 - e.r1, dz = e.z2 - e.z1;
        const float length = std::hypot(dr, dz);
        // control point of a quadratic curve: where the tangents at the ends meet
        float cr = (e.r1 + e.r2) / 2, cz = (e.z1 + e.z2) / 2;
        const float t1r = -e.n1.z, t1z = e.n1.r, t2r = -e.n2.z, t2z = e.n2.r;
        const float det = t1r * t2z - t1z * t2r;
        if (std::abs(det) > 1e-6f) {
            const float k = (dr * t2z - dz * t2r) / det;
            const float pr = e.r1 + k * t1r, pz = e.z1 + k * t1z;
            if (std::hypot(pr - cr, pz - cz) < 0.5f * length) {
                cr = pr;
                cz = pz;
            }
        }
        const int n = std::max(1, (int)std::ceil(2 * length / edge.step));
        for (int i = 0; i <= n; i++) {
            const float t = (float)i / n, w0 = (1 - t) * (1 - t), w1 = 2 * t * (1 - t), w2 = t * t;
            const float z = w0 * e.z1 + w1 * cz + w2 * e.z2;
            if (z <= top) {
                const float radius = std::max(0.0f, w0 * e.r1 + w1 * cr + w2 * e.r2);
                const float lowest = (float)(z * a[2] - radius * spread);
                slices.push_back({z, radius, lowest});
                lowestAll = std::min(lowestAll, lowest);
            }
        }
    }

    // the outline of every slice is a circle, projected an ellipse: its lowest point in every bin
    // it covers, exactly. Slices far above the lowest point don't make the edge near it.
    // across = cu + radius * acrossScale * cos(t - acrossPhase), up = cv + radius * (p2 cos t + q2 sin t)
    const double acrossScale = std::hypot(p[1], q[1]), acrossPhase = std::atan2(q[1], p[1]);
    const float reach = lowestAll + extent / 2;
    for (const Slice& slice : slices) {
        if (slice.lowest > reach) {
            continue;
        }
        const double cu = slice.z * a[1], cv = slice.z * a[2], radius = slice.radius;
        const double half = radius * acrossScale;
        if (half < edge.step / 2) {
            // seen edge on: a line up and down
            const int b = (int)std::lround((cu - edge.start) / edge.step);
            if (b >= 0 && b < bins) {
                edge.up[b] = std::min(edge.up[b], slice.lowest);
            }
            continue;
        }
        const int b0 = std::max(0, (int)std::ceil((cu - half - edge.start) / edge.step));
        const int b1 = std::min(bins - 1, (int)std::floor((cu + half - edge.start) / edge.step));
        for (int b = b0; b <= b1; b++) {
            const double w = std::clamp((edge.start + b * edge.step - cu) / half, -1.0, 1.0);
            const double d = std::acos(w);
            float lowest = std::numeric_limits<float>::infinity();
            for (const double t : {acrossPhase + d, acrossPhase - d}) {
                lowest = std::min(lowest, (float)(cv + radius * (p[2] * std::cos(t) + q[2] * std::sin(t))));
            }
            edge.up[b] = std::min(edge.up[b], lowest);
        }
    }
    return edge;
}

void CutterProfile::LineIntervals(
    const float origin[3],
    const float dir[3],
    std::vector<Interval>& out,
    std::vector<float>* normals,
    float sharpTolerance
) const
{
    out.clear();
    if (normals) {
        normals->clear();
    }
    const double ox = origin[0], oy = origin[1], oz = origin[2];
    const double ux = dir[0], uy = dir[1], uz = dir[2];
    struct Hit
    {
        double s;
        int edge;
    };
    constexpr int MaxHits = 256;
    Hit hits[MaxHits];
    int nHits = 0;
    const double horizontal = ux * ux + uy * uy;

    // the part of the line inside the bounding cylinder of the tool, and its heights
    double s0 = -1e300, s1 = 1e300;
    if (horizontal > 1e-12) {
        const double b = ox * ux + oy * uy;
        const double c = ox * ox + oy * oy - (double)mRadius * mRadius;
        const double disc = b * b - horizontal * c;
        if (disc < 0) {
            return;
        }
        const double sq = std::sqrt(disc);
        s0 = (-b - sq) / horizontal;
        s1 = (-b + sq) / horizontal;
    }
    else if (ox * ox + oy * oy > (double)mRadius * mRadius) {
        return;
    }
    double zmin = mMinZ, zmax = mMaxZ;
    if (std::abs(uz) > 1e-12) {
        const double za = oz + s0 * uz, zb = oz + s1 * uz;
        zmin = std::max(zmin, std::min(za, zb));
        zmax = std::min(zmax, std::max(za, zb));
    }
    else {
        zmin = std::max(zmin, oz);
        zmax = std::min(zmax, oz);
    }
    if (zmin > zmax + 1e-9) {
        return;
    }

    for (int edge = 0; edge < (int)mEdges.size() && nHits + 2 <= MaxHits; edge++) {
        const Edge& e = mEdges[edge];
        if (std::max(e.z1, e.z2) < zmin - 1e-9 || std::min(e.z1, e.z2) > zmax + 1e-9) {
            continue;
        }
        if (e.z1 == e.z2) {
            // annulus in a plane perpendicular to the axis
            if (std::abs(uz) < 1e-12) {
                continue;
            }
            const double s = (e.z1 - oz) / uz;
            const double x = ox + s * ux, y = oy + s * uy;
            const double rho2 = x * x + y * y;
            const double rlo = std::min(e.r1, e.r2), rhi = std::max(e.r1, e.r2);
            if (rho2 >= rlo * rlo && rho2 <= rhi * rhi) {
                hits[nHits++] = {s, edge};
            }
            continue;
        }
        const double zlo = std::min(e.z1, e.z2), zhi = std::max(e.z1, e.z2);
        // the line can only hit the cone where its height is in the range of the edge
        if (std::abs(uz) < 1e-12 && (oz < zlo || oz > zhi)) {
            continue;
        }
        // cone: rho(z) = r1 + k * (z - z1)
        const double k = (e.r2 - e.r1) / (e.z2 - e.z1);
        const double a0 = e.r1 + k * (oz - e.z1);
        const double a1 = k * uz;
        const double qa = horizontal - a1 * a1;
        const double qb = 2 * (ox * ux + oy * uy - a0 * a1);
        const double qc = ox * ox + oy * oy - a0 * a0;
        double roots[2];
        int n = 0;
        if (std::abs(qa) < 1e-12) {
            if (std::abs(qb) > 1e-12) {
                roots[n++] = -qc / qb;
            }
        }
        else {
            const double disc = qb * qb - 4 * qa * qc;
            if (disc >= 0) {
                const double sq = std::sqrt(disc);
                roots[n++] = (-qb - sq) / (2 * qa);
                roots[n++] = (-qb + sq) / (2 * qa);
            }
        }
        for (int i = 0; i < n; i++) {
            const double s = roots[i];
            const double z = oz + s * uz;
            if (z >= zlo && z <= zhi && a0 + a1 * s >= 0) {
                hits[nHits++] = {s, edge};
            }
        }
    }
    if (nHits < 2) {
        return;
    }

    // insertion sort, there are few hits
    for (int i = 1; i < nHits; i++) {
        const Hit h = hits[i];
        int j = i - 1;
        while (j >= 0 && hits[j].s > h.s) {
            hits[j + 1] = hits[j];
            j--;
        }
        hits[j + 1] = h;
    }
    int unique = 1;
    for (int i = 1; i < nHits; i++) {
        if (hits[i].s - hits[unique - 1].s >= 1e-9) {
            hits[unique++] = hits[i];
        }
    }
    nHits = unique;
    auto normalAt = [&](const Hit& hit, float* n) {
        const Edge& e = mEdges[hit.edge];
        const double x = ox + hit.s * ux, y = oy + hit.s * uy;
        const double rho = std::sqrt(x * x + y * y);
        const double z = oz + hit.s * uz;
        const ProfileNormal pn = NormalAt(e, (float)rho, (float)z);
        if (sharpTolerance > 0 && (e.sharp1 || e.sharp2)) {
            const double d1 = (rho - e.r1) * (rho - e.r1) + (z - e.z1) * (z - e.z1);
            const double d2 = (rho - e.r2) * (rho - e.r2) + (z - e.z2) * (z - e.z2);
            const double tolerance = (double)sharpTolerance * sharpTolerance;
            if ((e.sharp1 && d1 < tolerance) || (e.sharp2 && d2 < tolerance)) {
                n[0] = n[1] = n[2] = 0;
                return;
            }
        }
        n[0] = rho > 1e-9 ? (float)(pn.r * x / rho) : 0.0f;
        n[1] = rho > 1e-9 ? (float)(pn.r * y / rho) : 0.0f;
        n[2] = rho > 1e-9 ? pn.z : (pn.z < 0 ? -1.0f : 1.0f);
    };
    if (mConvex) {
        // one range from the first to the last hit
        hits[1] = hits[nHits - 1];
        nHits = 2;
    }
    for (int i = 0; i + 1 < nHits; i++) {
        const double mid = (hits[i].s + hits[i + 1].s) / 2;
        const double x = ox + mid * ux, y = oy + mid * uy;
        if (Contains((float)std::sqrt(x * x + y * y), (float)(oz + mid * uz))) {
            float n[3];
            if (!out.empty() && out.back().hi >= (float)hits[i].s) {
                out.back().hi = (float)hits[i + 1].s;
                if (normals) {
                    normalAt(hits[i + 1], n);
                    std::copy(n, n + 3, normals->end() - 3);
                }
            }
            else {
                out.push_back({(float)hits[i].s, (float)hits[i + 1].s});
                if (normals) {
                    normalAt(hits[i], n);
                    normals->insert(normals->end(), n, n + 3);
                    normalAt(hits[i + 1], n);
                    normals->insert(normals->end(), n, n + 3);
                }
            }
        }
    }
}

}  // namespace CAMSimulator
