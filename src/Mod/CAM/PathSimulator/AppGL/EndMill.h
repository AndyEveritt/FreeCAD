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

#pragma once

#include "SimShapes.h"
#include <vector>

#define PROFILE_BUFFER_POINTS(npoints) ((npoints) * 2 - 1)
#define PROFILE_BUFFER_SIZE(npoints) (PROFILE_BUFFER_POINTS(npoints) * 2)
#define MILL_HEIGHT 10

namespace CAMSimulator
{

class EndMill
{
public:
    std::vector<float> profilePoints;
    std::vector<float> rawProfile;  // the profile as given, radius/height pairs from top to tip
    float radius;
    int nPoints = 0;
    int toolId = -1;

    Shape pathShape;
    Shape halfToolShape;
    Shape toolShape;
    // the tool split where it cuts: the part that cuts, the parts above and below it
    Shape cuttingShape;
    Shape aboveShape;
    Shape belowShape;

public:
    EndMill(int toolid, float diameter);
    EndMill(const std::vector<float>& toolProfile, int toolid, float diameter);
    virtual ~EndMill();
    // The heights from the tip between which the tool cuts, to below 0 if unknown
    void SetCutting(float from, float to)
    {
        cutFrom = from;
        cutTo = to;
    }
    // Whether cuttingShape, aboveShape and belowShape split the tool
    bool HasCutting() const
    {
        return hasCutting;
    }
    void GenerateDisplayLists(float quality);
    unsigned int GenerateArcSegmentDL(float radius, float angleRad, float zShift, Shape* retShape) const;

protected:
    void MirrorPointBuffer();
    // the part of the profile between the heights lo and hi, closed on the axis, empty if none.
    // Horizontal edges at lo or hi belong to it only if 'closed'.
    std::vector<float> ClipProfile(float lo, float hi, bool closed) const;

    float cutFrom = 0;
    float cutTo = -1;
    bool hasCutting = false;
};

}  // namespace CAMSimulator
