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

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <TopoDS_Shape.hxx>

#include <Mod/CAM/App/Command.h>
#include <Mod/CAM/PathGlobal.h>
#include <Mod/Mesh/App/Mesh.h>
#include <Mod/Part/App/TopoShape.h>

#include "DlgCAMSimulator.h"

using namespace Path;

namespace CAMSimulator
{

/** The representation of a CNC Toolpath Simulator */

class CAMSimulatorExport CAMSim: public Base::BaseClass
{
    // TYPESYSTEM_HEADER();

public:
    static Base::Type getClassTypeId();
    Base::Type getTypeId() const override;
    static void init();
    static void* create();

private:
    static Base::Type classTypeId;


public:
    CAMSim() = default;

    void BeginSimulation(const Part::TopoShape& stock, float resolution);
    void resetSimulation(Gui::Document* doc);
    void addTool(
        const std::vector<float>& toolProfilePoints,
        int toolNumber,
        float diameter,
        float resolution,
        const std::vector<Material::ProfileArc>& arcs = {}
    );
    void SetBaseShape(const Part::TopoShape& baseShape, float resolution);
    void AddCommand(Command* cmd);
    void AddOperation(const std::string& name, const float* color = nullptr);

    void SetFrameTiming(bool enabled);
    std::vector<double> TakeFrameTimes();
    bool IsRenderPending();
    std::map<std::string, std::string> GetRendererInfo();
    void GoToOperation(int index);
    // the handler gets the index of the operation of a stock face clicked in the view, -1 for none
    void SetOperationPickedHandler(std::function<void(int)> handler);
    // the commands, numbered in the order they were added, see DlgCAMSimulator::goToCommand
    void GoToCommand(int index);
    void StepCommand(int direction);
    void SetCommandChangedHandler(std::function<void(int)> handler);
};

}  // namespace CAMSimulator
