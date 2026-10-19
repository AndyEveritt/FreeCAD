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


#include <Base/Console.h>
#include <Base/Interpreter.h>

#include "CAMSim.h"
#include "CAMSimPy.h"
#include "MaterialModel.h"
#include "ToolpathCheck.h"
#include "ViewCAMSimulator.h"

#include <Mod/CAM/App/CommandPy.h>
#include <Mod/Part/App/TopoShapePy.h>


namespace CAMSimulator
{

class Module: public Py::ExtensionModule<Module>
{
public:
    Module()
        : Py::ExtensionModule<Module>("CAMSimulator")
    {
        add_varargs_method(
            "checkToolpath",
            &Module::checkToolpath,
            "checkToolpath(steps, stock, model, quality, tolerance) -> list\n"
            "Simulate a toolpath without a view and return the problems found on it, as dicts like\n"
            "PathSim.GetEvents gives them, without their step. steps: the toolpath as the\n"
            "simulator is given it, ('command', command), ('operation', ...) and ('tool', profile,\n"
            "number, diameter, extras), see SimulationSteps; stock and model: shapes, model None\n"
            "for none; quality: 1 to 10; tolerance: of the distance from the model, mm."
        );
        initialize("This module is the CAMSimulator module.");  // register with Python
    }

    ~Module() override
    {}

private:
    // a closed triangle mesh of a shape
    static void Mesh(
        const Part::TopoShape& shape,
        double deflection,
        std::vector<float>& xyz,
        std::vector<uint32_t>& triangles
    )
    {
        std::vector<Base::Vector3d> points;
        std::vector<Data::ComplexGeoData::Facet> facets;
        shape.getFaces(points, facets, deflection);
        for (const Base::Vector3d& p : points) {
            xyz.insert(xyz.end(), {(float)p.x, (float)p.y, (float)p.z});
        }
        for (const auto& f : facets) {
            triangles.insert(triangles.end(), {f.I1, f.I2, f.I3});
        }
    }

    Py::Object checkToolpath(const Py::Tuple& args)
    {
        PyObject *steps, *stock, *model;
        float quality;
        double tolerance;
        if (!PyArg_ParseTuple(
                args.ptr(),
                "OO!Ofd",
                &steps,
                &(Part::TopoShapePy::Type),
                &stock,
                &model,
                &quality,
                &tolerance
            )) {
            throw Py::Exception();
        }
        ToolpathCheck check;
        for (const Py::Object& item : Py::Sequence(steps)) {
            const Py::Sequence step(item);
            const std::string kind = Py::String(step[0]).as_std_string();
            if (kind == "command") {
                if (!PyObject_TypeCheck(step[1].ptr(), &Path::CommandPy::Type)) {
                    throw Py::TypeError("a command step needs a Path.Command");
                }
                const Path::Command* cmd
                    = static_cast<Path::CommandPy*>(step[1].ptr())->getCommandPtr();
                check.AddCommand(cmd->Name, cmd->Parameters);
            }
            else if (kind == "operation") {
                check.BeginOperation();
            }
            else if (kind == "tool") {
                std::vector<float> profile;
                for (const Py::Object& v : Py::Sequence(step[1])) {
                    profile.push_back((float)Py::Float(v));
                }
                std::vector<Material::ProfileArc> arcs;
                double flutes = 0;
                if (step.size() > 4 && Py::Object(step[4]).isMapping()) {
                    const Py::Mapping extras(step[4]);
                    if (extras.hasKey("arcs") && !extras["arcs"].isNone()) {
                        for (const Py::Object& a : Py::Sequence(extras["arcs"])) {
                            const Py::Sequence arc(a);
                            arcs.push_back(
                                {(int)Py::Long(arc[0]),
                                 (int)Py::Long(arc[1]),
                                 (double)Py::Float(arc[2]),
                                 (double)Py::Float(arc[3])}
                            );
                        }
                    }
                    if (extras.hasKey("flutes")) {
                        flutes = Py::Float(extras["flutes"]);
                    }
                }
                check.AddTool((int)Py::Long(step[2]), profile, arcs, flutes);
            }
        }
        std::vector<float> xyz;
        std::vector<uint32_t> triangles;
        Mesh(*static_cast<Part::TopoShapePy*>(stock)->getTopoShapePtr(), 0.1, xyz, triangles);
        check.SetStock(xyz, triangles);
        if (model != Py_None) {
            if (!PyObject_TypeCheck(model, &Part::TopoShapePy::Type)) {
                throw Py::TypeError("model must be a shape or None");
            }
            // within 2 um, as the distance colors of the simulator
            std::vector<float> mxyz;
            std::vector<uint32_t> mtriangles;
            Mesh(*static_cast<Part::TopoShapePy*>(model)->getTopoShapePtr(), 0.002, mxyz, mtriangles);
            check.SetModel(std::make_shared<Material::ModelDistance>(mxyz, mtriangles, 1.0f), tolerance);
        }
        std::vector<ToolpathCheck::Found> found;
        Py_BEGIN_ALLOW_THREADS
        found = check.Run(quality);
        Py_END_ALLOW_THREADS

        static const char* kinds[] = {"rapid", "shank", "gouge", "left"};
        Py::List list;
        for (const ToolpathCheck::Found& f : found) {
            const Material::Event& e = f.event;
            Py::Dict d;
            d.setItem("kind", Py::String(kinds[e.kind]));
            d.setItem("command", Py::Long(f.command));
            d.setItem("operation", Py::Long(e.operation));
            d.setItem("volume", Py::Float(e.volume));
            d.setItem("depth", Py::Float(e.depth));
            d.setItem("count", Py::Long(e.count));
            d.setItem(
                "position",
                Py::TupleN(Py::Float(e.position[0]), Py::Float(e.position[1]), Py::Float(e.position[2]))
            );
            d.setItem(
                "box",
                Py::TupleN(
                    Py::TupleN(Py::Float(e.lo[0]), Py::Float(e.lo[1]), Py::Float(e.lo[2])),
                    Py::TupleN(Py::Float(e.hi[0]), Py::Float(e.hi[1]), Py::Float(e.hi[2]))
                )
            );
            list.append(d);
        }
        return list;
    }
};

PyObject* initModule()
{
    return Base::Interpreter().addModule(new Module);
}


}  // namespace CAMSimulator

/* Python entry */
PyMOD_INIT_FUNC(CAMSimulator)
{
    // load dependent module
    try {
        Base::Interpreter().runString("import Part");
        Base::Interpreter().runString("import Path");
        Base::Interpreter().runString("import Mesh");
    }
    catch (const Base::Exception& e) {
        PyErr_SetString(PyExc_ImportError, e.what());
        PyMOD_Return(nullptr);
    }

    //
    PyObject* mod = CAMSimulator::initModule();
    Base::Console().log("Loading CAMSimulator module.... done\n");

    // Add Types to module
    Base::Interpreter().addType(&CAMSimulator::CAMSimPy::Type, mod, "PathSim");

    // NOTE: To finish the initialization of our own type objects we must
    // call PyType_Ready, otherwise we run into a segmentation fault, later on.
    // This function is responsible for adding inherited slots from a type's base class.
    CAMSimulator::CAMSim::init();
    CAMSimulator::ViewCAMSimulator::init();

    PyMOD_Return(mod);
}
