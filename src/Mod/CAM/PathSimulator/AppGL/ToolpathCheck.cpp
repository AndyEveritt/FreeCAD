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
    // the axes of the plane of an arc: u, v, counterclockwise from u to v about their normal
    const int u = to.plane == 18 ? 2 : to.plane == 19 ? 1 : 0;
    const int v = to.plane == 18 ? 0 : to.plane == 19 ? 2 : 1;
    const double from3[3] = {from.x, from.y, from.z};
    const double to3[3] = {to.x, to.y, to.z};
    const double offset[3] = {to.i, to.j, to.k};
    // an arc around a center away from the start, as MillPathSegment has it
    cut.arc = (to.cmd == eRotateCW || to.cmd == eRotateCCW)
        && (std::abs(offset[u]) > 1e-5 || std::abs(offset[v]) > 1e-5);
    cut.rapid = to.rapid;
    cut.operation = to.operation;
    std::copy(from3, from3 + 3, cut.start);
    std::copy(to3, to3 + 3, cut.end);
    if (cut.arc) {
        for (int k = 0; k < 3; k++) {
            cut.center[k] = from3[k] + offset[k];
            cut.arcU[k] = k == u ? 1 : 0;
            cut.arcV[k] = k == v ? 1 : 0;
        }
        const double cu = cut.center[u], cv = cut.center[v];
        cut.radius = std::hypot(offset[u], offset[v]);
        const double direction = to.cmd == eRotateCCW ? -1 : 1;
        cut.startAngle = std::atan2(cu - from3[u], from3[v] - cv);
        const double endAngle = std::atan2(cu - to3[u], to3[v] - cv);
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
    double defaultRapid,
    const double* axisRapid
)
{
    // arcs: along their arc, and along their normal for helices
    double dz = std::abs(to.z - from.z);
    double horizontal = std::hypot(to.x - from.x, to.y - from.y);
    if (cut.arc) {
        const double n[3] = {
            cut.arcU[1] * cut.arcV[2] - cut.arcU[2] * cut.arcV[1],
            cut.arcU[2] * cut.arcV[0] - cut.arcU[0] * cut.arcV[2],
            cut.arcU[0] * cut.arcV[1] - cut.arcU[1] * cut.arcV[0],
        };
        horizontal = cut.radius * std::abs(cut.sweep);
        dz = std::abs(n[0] * (to.x - from.x) + n[1] * (to.y - from.y) + n[2] * (to.z - from.z));
    }
    const double turn = std::max({std::abs(to.a - from.a), std::abs(to.b - from.b), std::abs(to.c - from.c)});
    double seconds;
    if (to.rapid && rates.horizRapid <= 0 && axisRapid) {
        // the axes of the machine at their velocities
        const double d[3] = {std::abs(to.x - from.x), std::abs(to.y - from.y), dz};
        seconds = 0;
        for (int k = 0; k < 3; k++) {
            seconds = std::max(seconds, d[k] / (axisRapid[k] > 0 ? axisRapid[k] : defaultRapid));
        }
        seconds = std::max(seconds, turn / 90);
    }
    else if (to.rapid) {
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
    std::vector<MillMotion> ends;
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
        ends.push_back(motions[i]);
    }

    Material::Simulation sim;
    sim.SetFluteHeights(flutes);
    sim.SetModel(mModel, mTolerance);
    sim.Start(tools, segments, mStockXyz, mStockTriangles, quality);
    sim.WaitEvents();
    std::vector<Material::Event> events
        = ends.empty() ? std::vector<Material::Event>() : MachineEvents(mMachine, motions.front(), ends);
    const auto material = sim.Events();
    events.insert(events.end(), material.begin(), material.end());
    std::vector<Found> found;
    for (const Material::Event& e : events) {
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
