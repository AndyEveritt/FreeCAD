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

#include "GCodeParser.h"
#include "MachineKinematics.h"
#include "MaterialSimulation.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace CAMSimulator
{

// The segment of the material simulation for the motion from one position to the next: an arc
// in the XY plane if 'to' is one. Its tool and steps are left to the caller.
Material::Segment MotionSegment(const MillMotion& from, const MillMotion& to);

// The time of the machine for the motion of a segment, s: feed moves at their programmed feed
// rate, else at the feed rate of the tool (vertical for moves more vertical than horizontal),
// rapid moves at the rapid rates of the tool, else at the velocities of the X, Y and Z axes of the
// machine (axisRapid, mm/s, if given), else at defaultRapid, all axes at once: the longest of the
// axes. Turns of the tool take at least their largest angle at 30
// degrees a second feeding, 90 rapid.
double MotionSeconds(
    const MillMotion& from,
    const MillMotion& to,
    const Material::Segment& cut,
    const ToolRates& rates,
    double defaultRapid,
    const double* axisRapid = nullptr
);

// The problems of a toolpath (Material::Event), found by simulating it without a view: the
// toolpath as the simulator is given it, then Run.
class ToolpathCheck
{
public:
    // A tool, see Material::Tool: its profile, the arcs of its profile and its flute length
    void AddTool(
        int number,
        const std::vector<float>& profile,
        const std::vector<Material::ProfileArc>& arcs,
        double flutes
    );
    // The commands from now on are of a new operation
    void BeginOperation();
    // A command of the toolpath, see GCodeParser::AddCommand, numbered in the order they are added
    void AddCommand(const std::string& name, const std::map<std::string, double>& params);
    // The stock and the model, closed triangle meshes; without a model no regions are found
    void SetStock(std::vector<float> xyz, std::vector<uint32_t> triangles);
    void SetModel(std::shared_ptr<const Material::ModelDistance> model, double tolerance);
    // The machine of the toolpath, see MachineEvents
    void SetMachine(const MachineKinematics& machine)
    {
        mMachine = machine;
    }

    // A problem and the command where it starts (-1 for the regions of the final stock)
    struct Found
    {
        Material::Event event;
        int command = -1;
    };
    // Simulate the whole toolpath at a quality (1 to 10) and return the problems
    std::vector<Found> Run(float quality);

private:
    GCodeParser mParser;
    int mCommands = 0;
    int mOperations = 0;
    struct ToolData
    {
        std::vector<float> profile;
        std::vector<Material::ProfileArc> arcs;
        double flutes;
    };
    std::map<int, ToolData> mTools;
    std::vector<float> mStockXyz;
    std::vector<uint32_t> mStockTriangles;
    std::shared_ptr<const Material::ModelDistance> mModel;
    double mTolerance = 0.05;
    MachineKinematics mMachine;
};

}  // namespace CAMSimulator
