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

#include "MaterialSimulation.h"
#include "MillMotion.h"

#include <string>
#include <vector>

namespace CAMSimulator
{

// An axis of a machine: its name, its direction (linear) or the vector it turns about (rotary,
// right handed), its limits (mm or degrees), its velocity (mm/s or degrees/s), whether it moves
// the table (the part) or the head (the tool), and its place in the chain of its kind
struct MachineAxis
{
    std::string name;
    double vector[3] = {0, 0, 1};
    double min = -1e30;
    double max = 1e30;
    double velocity = 0;
    bool table = true;
    int sequence = 0;
};

// The axes of a machine (the Machine of a job), for its motions: which tool directions its rotary
// axes reach within their limits, how far its linear axes travel and how fast.
//
// Rotary axes of the table turn the part: a point of the part p is at R_n ... R_1 R_0 p in the
// machine, the table axis of sequence 0 nearest to the part (a C table carried by an A trunnion: C
// 0, A 1). Rotary axes of the head turn the tool: its direction is H_m ... H_1 H_0 z, the head axis
// of sequence 0 nearest to the tool. The tool direction in the part is R_0^T ... R_n^T H z. Up to
// two rotary axes are solved for a direction.
class MachineKinematics
{
public:
    bool Known() const
    {
        return mKnown;
    }
    // Linear axes X, Y, Z (by name, others are left out) and rotary axes, any order
    void SetAxes(const std::vector<MachineAxis>& linear, const std::vector<MachineAxis>& rotary);

    const std::vector<MachineAxis>& Rotaries() const
    {
        return mRotary;
    }
    // X, Y, Z, null if the machine doesn't name it
    const MachineAxis* Linear(int k) const
    {
        return mHasLinear[k] ? &mLinear[k] : nullptr;
    }

    // The tool direction in the part (unit) at the angles of the rotary axes (degrees, in the order
    // of Rotaries)
    void ToolAxis(const std::vector<double>& angles, double axis[3]) const;
    // A point of the part in the machine at the angles: turned by the table, about the origin
    void MachinePoint(const double p[3], const std::vector<double>& angles, double out[3]) const;
    // A point of the machine in the part at the angles, the inverse of MachinePoint
    void PartPoint(const double m[3], const std::vector<double>& angles, double out[3]) const;

    // The angles (degrees, in the order of Rotaries) that turn the tool along a direction of the
    // part, among them those within the limits of the axes if any (then 'withinLimits'), the one
    // nearest to 'near' (if not empty). False if the axes can't turn the tool there at all.
    bool AnglesFor(
        const double axis[3],
        const std::vector<double>& near,
        std::vector<double>& angles,
        bool& withinLimits
    ) const;

private:
    bool mKnown = false;
    MachineAxis mLinear[3];
    bool mHasLinear[3] = {false, false, false};
    std::vector<MachineAxis> mRotary;  // tables by sequence, then heads by sequence
};

// The motions of a toolpath the machine can't make: tool directions its rotary axes don't reach
// within their limits (Event::MachineLimit with no linear axis), and a toolpath wider than the
// travel of a linear axis (with the axis: what it needs and what it has). 'start' is where the
// toolpath starts, 'ends' the ends of its segments, in order. Moves turning the tool are left out:
// the machine turns its axes its own way, like between work planes.
std::vector<Material::Event> MachineEvents(
    const MachineKinematics& machine,
    const MillMotion& start,
    const std::vector<MillMotion>& ends
);

}  // namespace CAMSimulator
