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
            // on the axis: the tip of a cone is sharp, the middle of a flat face is not
            const ProfileNormal& m = onAxis(before) ? edgeNormals[i] : edgeNormals[before];
            sharp[i] = std::abs(m.r) > 0.01f;
        }
        else {
            sharp[i] = edgeNormals[before].r * edgeNormals[i].r + edgeNormals[before].z * edgeNormals[i].z < 0.94f;
        }
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
        mEdges.push_back({a.first, a.second, b.first, b.second, edgeNormals[i], sharp[i], sharp[(i + 1) % n]});
    }

    for (const auto& p : points) {
        mVertexHeights.push_back(p.second);
    }
    std::sort(mVertexHeights.begin(), mVertexHeights.end());
    mVertexHeights.erase(std::unique(mVertexHeights.begin(), mVertexHeights.end()), mVertexHeights.end());

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

    // every entry must be contained in the previous one
    for (int i = 1; i < TableSize && mMonotonic; i++) {
        for (int k = mTableStart[i]; k < mTableStart[i + 1]; k++) {
            const Interval& inner = mTableIntervals[k];
            bool contained = false;
            for (int j = mTableStart[i - 1]; j < mTableStart[i]; j++) {
                const Interval& outer = mTableIntervals[j];
                contained |= outer.lo <= inner.lo + 1e-5f && inner.hi <= outer.hi + 1e-5f;
            }
            mMonotonic &= contained;
        }
    }
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
                normals->push_back(mEdges[crossings[i].second].normal);
                normals->push_back(mEdges[crossings[i + 1].second].normal);
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

float CutterProfile::SliceRadius(float z, ProfileNormal& normal) const
{
    float best = -1;
    for (const Edge& e : mEdges) {
        const float zlo = std::min(e.z1, e.z2), zhi = std::max(e.z1, e.z2);
        if (z < zlo || z > zhi) {
            continue;
        }
        const float r = e.z1 == e.z2 ? std::max(e.r1, e.r2)
                                     : e.r1 + (z - e.z1) * (e.r2 - e.r1) / (e.z2 - e.z1);
        // at a vertex shared by two edges, prefer the edge facing outwards
        if (r > best || (r == best && e.normal.r > normal.r)) {
            best = r;
            normal = e.normal;
        }
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

float CutterProfile::FeatureSize() const
{
    ProfileNormal normal;
    const float r = SliceRadius(std::min(mRadius, mMaxZ - mMinZ) / 2 + mMinZ, normal);
    return std::clamp(r, mRadius / 4, mRadius);
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
        const ProfileNormal& pn = e.normal;
        const double x = ox + hit.s * ux, y = oy + hit.s * uy;
        const double rho = std::sqrt(x * x + y * y);
        if (sharpTolerance > 0 && (e.sharp1 || e.sharp2)) {
            const double z = oz + hit.s * uz;
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
