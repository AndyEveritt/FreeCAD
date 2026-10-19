// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2024 Shai Seger <shaise at gmail>                       *
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

#include "EndMill.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

EndMill::EndMill(int toolid, float diameter)
{
    radius = diameter / 2;
    toolId = toolid;
}

EndMill::EndMill(const std::vector<float>& toolProfile, int toolid, float diameter)
    : EndMill(toolid, diameter)
{
    rawProfile = toolProfile;
    profilePoints.clear();

    int srcBuffSize = toolProfile.size();
    nPoints = srcBuffSize / 2;
    if (nPoints < 2) {
        return;
    }

    // make sure last point is at 0,0 else, add it
    bool missingCenterPoint = fabs(toolProfile[nPoints * 2 - 2]) > 0.0001f;
    if (missingCenterPoint) {
        nPoints++;
    }

    int buffSize = PROFILE_BUFFER_SIZE(nPoints);
    profilePoints.resize(buffSize);

    // copy profile points
    for (int i = 0; i < srcBuffSize; i += 2) {
        // add some width to reduce simulation artifacts
        profilePoints[i] = toolProfile[i] + diameter * 0.01f;
        profilePoints[i + 1] = toolProfile[i + 1] - diameter * 0.01f;
    }
    if (missingCenterPoint) {
        profilePoints[srcBuffSize] = 0.0F;
        profilePoints[srcBuffSize + 1] = profilePoints[srcBuffSize - 1];
    }

    MirrorPointBuffer();
}

EndMill::~EndMill()
{
    toolShape.FreeResources();
    cuttingShape.FreeResources();
    aboveShape.FreeResources();
    belowShape.FreeResources();
    halfToolShape.FreeResources();
    pathShape.FreeResources();
}

void EndMill::GenerateDisplayLists(float quality)
{
    // calculate number of slices based on quality.
    int nslices = 16;
    if (quality < 3) {
        nslices = 4;
    }
    else if (quality < 7) {
        nslices = 8;
    }

    // full tool
    toolShape.RotateProfile(profilePoints.data(), nPoints, 0, 0, nslices, false);
    // the part that cuts and the rest, by height from the tip
    hasCutting = false;
    if (cutTo >= 0 && nPoints >= 2) {
        float tip = profilePoints[1], top = profilePoints[1];
        for (int i = 1; i < nPoints; i++) {
            tip = std::min(tip, profilePoints[i * 2 + 1]);
            top = std::max(top, profilePoints[i * 2 + 1]);
        }
        const float lo = tip + cutFrom, hi = tip + cutTo;
        const auto rotate = [&](Shape& shape, const std::vector<float>& profile) {
            shape.FreeResources();
            if (profile.size() >= 4) {
                shape.RotateProfile(profile.data(), (int)profile.size() / 2, 0, 0, nslices, false);
            }
        };
        rotate(cuttingShape, ClipProfile(lo, hi, true));
        rotate(aboveShape, ClipProfile(hi, top, false));
        rotate(belowShape, ClipProfile(tip, lo, false));
        hasCutting = true;
    }

    // half tool
    halfToolShape.RotateProfile(profilePoints.data(), nPoints, 0, 0, nslices / 2, true);

    // unit path
    int nFullPoints = PROFILE_BUFFER_POINTS(nPoints);
    pathShape.ExtrudeProfileLinear(profilePoints.data(), nFullPoints, 0, 1, 0, 0, true, false);
}

unsigned int EndMill::GenerateArcSegmentDL(float radius, float angleRad, float zShift, Shape* retShape) const
{
    int nFullPoints = PROFILE_BUFFER_POINTS(nPoints);
    retShape->ExtrudeProfileRadial(profilePoints.data(), nFullPoints, radius, angleRad, zShift, true, true);
    return 0;
}

std::vector<float> EndMill::ClipProfile(float lo, float hi, bool closed) const
{
    // the profile goes from the top down to the tip, its heights don't rise
    std::vector<float> out;
    const auto add = [&out](float r, float z) {
        const size_t n = out.size();
        if (n < 2 || std::abs(out[n - 2] - r) > 1e-6f || std::abs(out[n - 1] - z) > 1e-6f) {
            out.insert(out.end(), {r, z});
        }
    };
    if (hi - lo <= 1e-6f) {
        return out;
    }
    for (int i = 1; i < nPoints; i++) {
        const float r0 = profilePoints[i * 2 - 2], z0 = profilePoints[i * 2 - 1];
        const float r1 = profilePoints[i * 2], z1 = profilePoints[i * 2 + 1];
        if (std::abs(z0 - z1) <= 1e-6f) {
            // a horizontal edge: within the heights, or at one of them if closed
            const bool inside = closed ? z0 >= lo - 1e-6f && z0 <= hi + 1e-6f
                                       : z0 > lo + 1e-6f && z0 < hi - 1e-6f;
            if (inside) {
                add(r0, z0);
                add(r1, z1);
            }
            continue;
        }
        const float top = std::min(z0, hi), bottom = std::max(z1, lo);
        if (top - bottom <= 1e-6f) {
            continue;
        }
        const auto at = [&](float z) {
            return r0 + (r1 - r0) * (z0 - z) / (z0 - z1);
        };
        add(at(top), top);
        add(at(bottom), bottom);
    }
    // closed on the axis at the bottom, the top is closed by RotateProfile
    if (out.size() >= 4 && out[out.size() - 2] > 1e-6f) {
        add(0, out.back());
    }
    if (out.size() < 4) {
        out.clear();
    }
    return out;
}

void EndMill::MirrorPointBuffer()
{
    int endpoint = PROFILE_BUFFER_POINTS(nPoints) - 1;
    for (int i = 0, j = endpoint * 2; i < (nPoints - 1) * 2; i += 2, j -= 2) {
        profilePoints[j] = -profilePoints[i];
        profilePoints[j + 1] = profilePoints[i + 1];
    }
}

}  // namespace CAMSimulator
