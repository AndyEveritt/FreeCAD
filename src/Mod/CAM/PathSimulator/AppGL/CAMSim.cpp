// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2017 Shai Seger <shaise at gmail>                       *
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


#include "CAMSim.h"

#include <unordered_map>

#include "DlgCAMSimulator.h"
#include <string>
#include <vector>

using namespace Base;

TYPESYSTEM_SOURCE(CAMSimulator::CAMSim, Base::BaseClass);

namespace CAMSimulator
{

void CAMSim::BeginSimulation(const Part::TopoShape& stock, float quality)
{
    DlgCAMSimulator::instance()->startSimulation(stock, quality);
}

void CAMSim::resetSimulation(Gui::Document* doc)
{
    DlgCAMSimulator::instance(doc)->resetSimulation();
}

void CAMSim::addTool(
    const std::vector<float>& toolProfilePoints,
    int toolNumber,
    float diameter,
    float resolution,
    const std::vector<Material::ProfileArc>& arcs,
    double flutes,
    const ToolRates& rates
)
{
    DlgCAMSimulator::instance()
        ->addTool(toolProfilePoints, toolNumber, diameter, resolution, arcs, flutes, rates);
}

Mesh::MeshObject* CAMSim::GetStockMesh()
{
    std::vector<float> xyz;
    std::vector<uint32_t> triangles;
    DlgCAMSimulator::instance()->stockTriangles(xyz, triangles);
    // the vertices of the parts of the mesh at the same place are one
    std::unordered_map<std::string, uint32_t> indexOf;
    std::vector<uint32_t> merged(xyz.size() / 3);
    MeshCore::MeshPointArray points;
    for (size_t i = 0; i < merged.size(); i++) {
        const std::string key(reinterpret_cast<const char*>(&xyz[3 * i]), 3 * sizeof(float));
        const auto [it, added] = indexOf.emplace(key, (uint32_t)points.size());
        if (added) {
            points.push_back(MeshCore::MeshPoint(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]));
        }
        merged[i] = it->second;
    }
    MeshCore::MeshFacetArray facets;
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const uint32_t a = merged[triangles[i]], b = merged[triangles[i + 1]],
                       c = merged[triangles[i + 2]];
        if (a != b && b != c && a != c) {
            facets.push_back(MeshCore::MeshFacet(a, b, c));
        }
    }
    MeshCore::MeshKernel kernel;
    kernel.Adopt(points, facets, true);
    return new Mesh::MeshObject(kernel);
}

void CAMSim::SetMachine(const MachineKinematics& machine)
{
    DlgCAMSimulator::instance()->setMachine(machine);
}

double CAMSim::GetTotalSeconds()
{
    return DlgCAMSimulator::instance()->totalSeconds();
}

std::vector<double> CAMSim::GetOperationSeconds()
{
    return DlgCAMSimulator::instance()->operationSeconds();
}

void CAMSim::SetBaseShape(const Part::TopoShape& baseShape, float resolution)
{
    if (baseShape.isNull()) {
        return;
    }

    DlgCAMSimulator::instance()->setBaseShape(baseShape, resolution);
}

void CAMSim::AddCommand(Command* cmd)
{
    DlgCAMSimulator::instance()->addCommand(cmd->Name, cmd->Parameters);
}

void CAMSim::AddOperation(const std::string& name, const float* color)
{
    DlgCAMSimulator::instance()->addOperation(name.c_str(), color);
}

void CAMSim::SetFrameTiming(bool enabled)
{
    DlgCAMSimulator::instance()->setFrameTiming(enabled);
}

std::vector<double> CAMSim::TakeFrameTimes()
{
    return DlgCAMSimulator::instance()->takeFrameTimes();
}

bool CAMSim::IsRenderPending()
{
    return DlgCAMSimulator::instance()->isRenderPending();
}

std::map<std::string, std::string> CAMSim::GetRendererInfo()
{
    return DlgCAMSimulator::instance()->rendererInfo();
}

void CAMSim::GoToOperation(int index)
{
    DlgCAMSimulator::instance()->goToOperation(index);
}

void CAMSim::SetOperationPickedHandler(std::function<void(int)> handler)
{
    DlgCAMSimulator::instance()->setOperationPickedHandler(std::move(handler));
}

std::vector<SimEvent> CAMSim::GetEvents()
{
    return DlgCAMSimulator::instance()->events();
}

void CAMSim::GoToEvent(int index)
{
    DlgCAMSimulator::instance()->goToEvent(index);
}

void CAMSim::SetEventsChangedHandler(std::function<void(int)> handler)
{
    DlgCAMSimulator::instance()->setEventsChangedHandler(std::move(handler));
}

void CAMSim::GoToCommand(int index)
{
    DlgCAMSimulator::instance()->goToCommand(index);
}

void CAMSim::StepCommand(int direction)
{
    DlgCAMSimulator::instance()->stepCommand(direction);
}

void CAMSim::SetCommandChangedHandler(std::function<void(int)> handler)
{
    DlgCAMSimulator::instance()->setCommandChangedHandler(std::move(handler));
}

}  // namespace CAMSimulator
