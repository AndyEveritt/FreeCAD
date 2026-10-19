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

// The machine and the problems of the simulator in Python

#pragma once

#include "MachineKinematics.h"

#include <CXX/Objects.hxx>

namespace CAMSimulator
{

// A machine from its Python description, None for none: {"linear": [(name, min, max, velocity)],
// "rotary": [(name, (x, y, z), min, max, velocity, table, sequence)]}, mm, degrees, per second
inline MachineKinematics MachineFromPython(const Py::Object& object)
{
    MachineKinematics machine;
    if (object.isNone()) {
        return machine;
    }
    const Py::Mapping description(object);
    std::vector<MachineAxis> linear, rotary;
    if (description.hasKey("linear")) {
        for (const Py::Object& item : Py::Sequence(description["linear"])) {
            const Py::Sequence s(item);
            MachineAxis a;
            a.name = Py::String(s[0]).as_std_string();
            a.min = Py::Float(s[1]);
            a.max = Py::Float(s[2]);
            a.velocity = Py::Float(s[3]);
            linear.push_back(a);
        }
    }
    if (description.hasKey("rotary")) {
        for (const Py::Object& item : Py::Sequence(description["rotary"])) {
            const Py::Sequence s(item);
            MachineAxis a;
            a.name = Py::String(s[0]).as_std_string();
            const Py::Sequence v(s[1]);
            for (int k = 0; k < 3; k++) {
                a.vector[k] = Py::Float(v[k]);
            }
            a.min = Py::Float(s[2]);
            a.max = Py::Float(s[3]);
            a.velocity = Py::Float(s[4]);
            a.table = Py::Boolean(s[5]);
            a.sequence = (int)Py::Long(s[6]);
            rotary.push_back(a);
        }
    }
    machine.SetAxes(linear, rotary);
    return machine;
}

// A problem of the simulator as a dict, see CAMSim.GetEvents
inline Py::Dict EventToPython(const Material::Event& e)
{
    static const char* kinds[] = {"rapid", "shank", "gouge", "left", "machine"};
    auto triple = [](const double v[3]) {
        return Py::TupleN(Py::Float(v[0]), Py::Float(v[1]), Py::Float(v[2]));
    };
    Py::Dict d;
    d.setItem("kind", Py::String(kinds[e.kind]));
    d.setItem("operation", Py::Long(e.operation));
    d.setItem("volume", Py::Float(e.volume));
    d.setItem("position", triple(e.position));
    d.setItem("depth", Py::Float(e.depth));
    d.setItem("box", Py::TupleN(triple(e.lo), triple(e.hi)));
    d.setItem("count", Py::Long(e.count));
    d.setItem("axis", Py::String(e.axis));
    d.setItem("needed", Py::Float(e.needed));
    d.setItem("available", Py::Float(e.available));
    d.setItem("direction", triple(e.direction));
    return d;
}

}  // namespace CAMSimulator
