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

#include "ToolpathCheck.h"

#include <algorithm>
#include <cmath>

namespace CAMSimulator
{

Material::Segment MotionSegment(const MillMotion& from, const MillMotion& to)
{
    Material::Segment cut;
    // an arc around a center away from the start, as MillPathSegment has it
    cut.arc = (to.cmd == eRotateCW || to.cmd == eRotateCCW)
        && (std::abs(to.i) > 1e-5 || std::abs(to.j) > 1e-5);
    cut.rapid = to.rapid;
    cut.operation = to.operation;
    cut.start[0] = from.x;
    cut.start[1] = from.y;
    cut.start[2] = from.z;
    cut.end[0] = to.x;
    cut.end[1] = to.y;
    cut.end[2] = to.z;
    if (cut.arc) {
        cut.center[0] = from.x + to.i;
        cut.center[1] = from.y + to.j;
        cut.radius = std::hypot(to.i, to.j);
        const double direction = to.cmd == eRotateCCW ? -1 : 1;
        cut.startAngle = std::atan2(cut.center[0] - from.x, from.y - cut.center[1]);
        const double endAngle = std::atan2(cut.center[0] - to.x, to.y - cut.center[1]);
        double sweep = (cut.startAngle - endAngle) * direction;
        if (sweep < 1e-5) {
            sweep += 2 * M_PI;  // a full circle
        }
        cut.sweep = -direction * sweep;
    }
    MotionToolAxis(from, cut.axis0);
    MotionToolAxis(to, cut.axis1);
    return cut;
}

// The time of the machine for a motion of a segment, s. Feed moves at their programmed feed rate,
// else at the feed rate of the tool, rapid moves at its rapid rates, the axes at once: the longer
// of the horizontal and the vertical. Turns of the tool take at least their largest angle at 30
// degrees a second feeding, 90 rapid.
double MotionSeconds(
    const MillMotion& from,
    const MillMotion& to,
    const Material::Segment& cut,
    const ToolRates& rates,
    double defaultRapid
)
{
    const double dz = std::abs(to.z - from.z);
    const double horizontal = cut.arc ? cut.radius * std::abs(cut.sweep)
                                      : std::hypot(to.x - from.x, to.y - from.y);
    const double turn = std::max({std::abs(to.a - from.a), std::abs(to.b - from.b), std::abs(to.c - from.c)});
    double seconds;
    if (to.rapid) {
        const double h = rates.horizRapid > 0 ? rates.horizRapid : defaultRapid;
        const double v = rates.vertRapid > 0 ? rates.vertRapid : h;
        seconds = std::max(horizontal / h, dz / v);
        seconds = std::max(seconds, turn / 90);
    }
    else {
        double feed = to.feed;
        if (feed <= 0) {
            feed = dz > horizontal && rates.vertFeed > 0 ? rates.vertFeed : rates.horizFeed;
        }
        if (feed <= 0) {
            feed = rates.vertFeed > 0 ? rates.vertFeed : 1000.0 / 60;
        }
        seconds = std::hypot(horizontal, dz) / feed;
        seconds = std::max(seconds, turn / 30);
    }
    return seconds;
}

void ToolpathCheck::AddTool(
    int number,
    const std::vector<float>& profile,
    const std::vector<Material::ProfileArc>& arcs,
    double flutes
)
{
    mTools[number] = {profile, arcs, flutes};
    mParser.AddCommand("T" + std::to_string(number), {});
}

void ToolpathCheck::BeginOperation()
{
    mParser.currentOperation = mOperations++;
}

void ToolpathCheck::AddCommand(const std::string& name, const std::map<std::string, double>& params)
{
    mParser.AddCommand(name, params, mCommands++);
}

void ToolpathCheck::SetStock(std::vector<float> xyz, std::vector<uint32_t> triangles)
{
    mStockXyz = std::move(xyz);
    mStockTriangles = std::move(triangles);
}

void ToolpathCheck::SetModel(std::shared_ptr<const Material::ModelDistance> model, double tolerance)
{
    mModel = std::move(model);
    mTolerance = tolerance;
}

std::vector<ToolpathCheck::Found> ToolpathCheck::Run(float quality)
{
    mParser.FillStart();
    // the tools by index, the segments of the motions of known tools
    std::vector<std::shared_ptr<const Material::Tool>> tools;
    std::vector<double> flutes;
    std::map<int, int> indexOf;
    for (const auto& [number, tool] : mTools) {
        indexOf[number] = (int)tools.size();
        tools.push_back(std::make_shared<Material::Tool>(tool.profile, true, tool.arcs));
        flutes.push_back(tool.flutes);
    }
    std::vector<Material::Segment> segments;
    std::vector<int> sources;
    const auto& motions = mParser.Operations;
    for (size_t i = 1; i < motions.size(); i++) {
        const auto tool = indexOf.find(motions[i].tool);
        if (tool == indexOf.end()) {
            continue;
        }
        Material::Segment s = MotionSegment(motions[i - 1], motions[i]);
        s.tool = tool->second;
        segments.push_back(s);
        sources.push_back(motions[i].source);
    }

    Material::Simulation sim;
    sim.SetFluteHeights(flutes);
    sim.SetModel(mModel, mTolerance);
    sim.Start(tools, segments, mStockXyz, mStockTriangles, quality);
    sim.WaitEvents();
    std::vector<Found> found;
    for (const Material::Event& e : sim.Events()) {
        Found f;
        f.event = e;
        if (e.firstSegment >= 0 && e.firstSegment < (int)sources.size()) {
            f.command = sources[e.firstSegment];
        }
        found.push_back(f);
    }
    return found;
}

}  // namespace CAMSimulator
