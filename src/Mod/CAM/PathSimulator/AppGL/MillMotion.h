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

#include "linmath.h"

#include <cmath>

namespace CAMSimulator
{

enum eEndMillType
{
    eEndmillFlat,
    eEndmillV,
    eEndmillBall,
    eEndmillFillet
};

enum eCmdType
{
    eNop,
    eMoveLiner,
    eRotateCW,
    eRotateCCW,
    eChangeTool
};

// A motion of the toolpath to a position: tool tip positions in workpiece coordinates (tool center
// point programming), in mm
struct MillMotion
{
    eCmdType cmd = eNop;
    int tool = -1;
    double x = 0, y = 0, z = 0;
    double i = 0, j = 0, k = 0;  // arcs: center relative to the start
    // rotary axes in degrees
    double a = 0, b = 0, c = 0;
    bool rapid = false;  // G0
    double feed = 0;     // programmed feed rate in mm/s, 0 if none
    int operation = -1;  // index of the operation the motion belongs to, -1 for none
    int source = -1;     // index of the toolpath command the motion comes from, -1 for none
};

// Tool axis (unit vector from the tip towards the shank) in workpiece coordinates. Positions are
// tool tip positions in workpiece coordinates (tool center point programming), and the rotary
// axes orient the tool: the vertical axis rotated by A around X, then B around Y, then C around Z.
// This is the single place to adapt for machine specific kinematics.
static inline void MotionToolAxis(const MillMotion& motion, double axis[3])
{
    const double deg = 3.14159265358979323846 / 180.0;
    const double a = motion.a * deg, b = motion.b * deg, c = motion.c * deg;
    // Rx(a) * (0, 0, 1)
    const double x1 = 0, y1 = -std::sin(a), z1 = std::cos(a);
    // Ry(b)
    const double x2 = x1 * std::cos(b) + z1 * std::sin(b), y2 = y1;
    const double z2 = -x1 * std::sin(b) + z1 * std::cos(b);
    // Rz(c)
    axis[0] = x2 * std::cos(c) - y2 * std::sin(c);
    axis[1] = x2 * std::sin(c) + y2 * std::cos(c);
    axis[2] = z2;
}

static inline void MotionPosToVec(vec3 vec, const MillMotion& motion)
{
    vec[0] = (float)motion.x;
    vec[1] = (float)motion.y;
    vec[2] = (float)motion.z;
}

}  // namespace CAMSimulator
