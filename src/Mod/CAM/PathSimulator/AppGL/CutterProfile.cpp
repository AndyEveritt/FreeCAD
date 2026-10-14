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

    mMinZ = mMaxZ = points.front().second;
    for (size_t i = 0; i < points.size(); i++) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % points.size()];
        mRadius = std::max(mRadius, a.first);
        mMinZ = std::min(mMinZ, a.second);
        mMaxZ = std::max(mMaxZ, a.second);
        if (a.first == 0 && b.first == 0) {
            continue;  // on the axis
        }
        mEdges.push_back({a.first, a.second, b.first, b.second});
    }

    // table of axial intervals
    mTableStep = mRadius > 0 ? mRadius / (TableSize - 1) : 1;
    std::vector<Interval> intervals;
    mTableStart.reserve(TableSize + 1);
    for (int i = 0; i < TableSize; i++) {
        mTableStart.push_back((int)mTableIntervals.size());
        ComputeAxialIntervals(i * mTableStep, intervals);
        mTableIntervals.insert(mTableIntervals.end(), intervals.begin(), intervals.end());
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

void CutterProfile::ComputeAxialIntervals(float d, std::vector<Interval>& out) const
{
    out.clear();
    std::vector<float> crossings;
    for (const Edge& e : mEdges) {
        // half open, so a vertex is counted once
        if ((e.r1 <= d) != (e.r2 <= d)) {
            crossings.push_back(e.z1 + (d - e.r1) * (e.z2 - e.z1) / (e.r2 - e.r1));
        }
    }
    std::sort(crossings.begin(), crossings.end());
    for (size_t i = 0; i + 1 < crossings.size(); i += 2) {
        if (crossings[i + 1] > crossings[i]) {
            out.push_back({crossings[i], crossings[i + 1]});
        }
    }
}

int CutterProfile::AxialIntervals(float d, const Interval*& out) const
{
    const int index = (int)std::ceil(d / mTableStep);
    if (index < 0 || index >= TableSize) {
        out = nullptr;
        return 0;
    }
    out = mTableIntervals.data() + mTableStart[index];
    return mTableStart[index + 1] - mTableStart[index];
}

void CutterProfile::LineIntervals(const float origin[3], const float dir[3], std::vector<Interval>& out) const
{
    out.clear();
    const double ox = origin[0], oy = origin[1], oz = origin[2];
    const double ux = dir[0], uy = dir[1], uz = dir[2];
    std::vector<double> hits;

    for (const Edge& e : mEdges) {
        const double zlo = std::min(e.z1, e.z2), zhi = std::max(e.z1, e.z2);
        if (e.z1 == e.z2) {
            // annulus in a plane perpendicular to the axis
            if (std::abs(uz) < 1e-12) {
                continue;
            }
            const double s = (e.z1 - oz) / uz;
            const double rho = std::hypot(ox + s * ux, oy + s * uy);
            if (rho >= std::min(e.r1, e.r2) && rho <= std::max(e.r1, e.r2)) {
                hits.push_back(s);
            }
            continue;
        }
        // cone: rho(z) = r1 + k * (z - z1)
        const double k = (e.r2 - e.r1) / (e.z2 - e.z1);
        const double a0 = e.r1 + k * (oz - e.z1);
        const double a1 = k * uz;
        const double qa = ux * ux + uy * uy - a1 * a1;
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
                hits.push_back(s);
            }
        }
    }

    std::sort(hits.begin(), hits.end());
    hits.erase(
        std::unique(hits.begin(), hits.end(), [](double a, double b) { return b - a < 1e-9; }),
        hits.end()
    );
    for (size_t i = 0; i + 1 < hits.size(); i++) {
        const double mid = (hits[i] + hits[i + 1]) / 2;
        const double rho = std::hypot(ox + mid * ux, oy + mid * uy);
        if (Contains((float)rho, (float)(oz + mid * uz))) {
            if (!out.empty() && out.back().hi >= (float)hits[i]) {
                out.back().hi = (float)hits[i + 1];
            }
            else {
                out.push_back({(float)hits[i], (float)hits[i + 1]});
            }
        }
    }
}

}  // namespace CAMSimulator
