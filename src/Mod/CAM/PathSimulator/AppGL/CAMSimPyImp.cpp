// SPDX-License-Identifier: LGPL-2.1-or-later

/**************************************************************************
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


#include <Base/Interpreter.h>
#include <Base/PlacementPy.h>
#include <Base/PyWrapParseTupleAndKeywords.h>

#include <Gui/Document.h>
#include <Mod/Mesh/App/MeshPy.h>
#include <Mod/CAM/App/CommandPy.h>
#include <Mod/Part/App/TopoShapePy.h>

#include "DocumentPy.h"
// inclusion of the generated files (generated out of CAMSimPy.xml)
#include "CAMSimPy.h"
#include "CAMSimPy.cpp"


namespace CAMSimulator
{

// returns a string which represents the object e.g. when printed in python
std::string CAMSimPy::representation() const
{
    return std::string("<CAMSim object>");
}

PyObject* CAMSimPy::PyMake(struct _typeobject*, PyObject*, PyObject*)  // Python wrapper
{
    // create a new instance of CAMSimPy and the Twin object
    return new CAMSimPy(new CAMSim);
}

// constructor method
int CAMSimPy::PyInit(PyObject* /*args*/, PyObject* /*kwd*/)
{
    return 0;
}


PyObject* CAMSimPy::ResetSimulation(PyObject* args)
{
    PyObject* pObjDoc;
    if (!PyArg_ParseTuple(args, "O!", &(Gui::DocumentPy::Type), &pObjDoc)) {
        return nullptr;
    }
    CAMSim* sim = getCAMSimPtr();
    Gui::Document* doc = static_cast<Gui::DocumentPy*>(pObjDoc)->getDocumentPtr();
    sim->resetSimulation(doc);
    Py_IncRef(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::BeginSimulation(PyObject* args, PyObject* kwds)
{
    static const std::array<const char*, 3> kwlist {"stock", "resolution", nullptr};
    PyObject* pObjStock;
    float resolution;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "O!f",
            kwlist,
            &(Part::TopoShapePy::Type),
            &pObjStock,
            &resolution
        )) {
        return nullptr;
    }
    CAMSim* sim = getCAMSimPtr();
    const Part::TopoShape& stock = *static_cast<Part::TopoShapePy*>(pObjStock)->getTopoShapePtr();
    sim->BeginSimulation(stock, resolution);
    Py_IncRef(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::AddTool(PyObject* args, PyObject* kwds)
{
    static const std::array<const char*, 9> kwlist {
        "shape",
        "toolnumber",
        "diameter",
        "resolution",
        "arcs",
        "flutes",
        "feeds",
        "rapids",
        nullptr
    };
    PyObject* pObjToolShape;
    int toolNumber;
    float resolution;
    float diameter;
    PyObject* pyArcs = Py_None;
    double flutes = 0;
    ToolRates rates;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "Oiff|Od(dd)(dd)",
            kwlist,
            &pObjToolShape,
            &toolNumber,
            &diameter,
            &resolution,
            &pyArcs,
            &flutes,
            &rates.horizFeed,
            &rates.vertFeed,
            &rates.horizRapid,
            &rates.vertRapid
        )) {
        return nullptr;
    }
    // the arcs of the profile: (first, last, r, z)
    std::vector<Material::ProfileArc> arcs;
    if (pyArcs != Py_None) {
        PyObject* seq = PySequence_Fast(pyArcs, "arcs must be a sequence of (first, last, r, z)");
        if (!seq) {
            return nullptr;
        }
        const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
        for (Py_ssize_t i = 0; i < n; i++) {
            Material::ProfileArc arc;
            if (!PyArg_ParseTuple(
                    PySequence_Fast_GET_ITEM(seq, i),
                    "iidd",
                    &arc.first,
                    &arc.last,
                    &arc.r,
                    &arc.z
                )) {
                Py_DECREF(seq);
                return nullptr;
            }
            arcs.push_back(arc);
        }
        Py_DECREF(seq);
    }
    // The tool shape is defined by a list of 2d points that represents the tool revolving profile
    Py_ssize_t num_floats = PyList_Size(pObjToolShape);
    std::vector<float> toolProfile;
    for (Py_ssize_t i = 0; i < num_floats; ++i) {
        PyObject* item = PyList_GetItem(pObjToolShape, i);
        toolProfile.push_back(static_cast<float>(PyFloat_AsDouble(item)));
    }

    CAMSim* sim = getCAMSimPtr();
    sim->addTool(toolProfile, toolNumber, diameter, resolution, arcs, flutes, rates);

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::SetBaseShape(PyObject* args, PyObject* kwds)
{
    static const std::array<const char*, 3> kwlist {"shape", "resolution", nullptr};
    PyObject* pObjBaseShape;
    float resolution;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "O!f",
            kwlist,
            &(Part::TopoShapePy::Type),
            &pObjBaseShape,
            &resolution
        )) {
        return nullptr;
    }
    if (!PyArg_ParseTuple(args, "O!f", &(Part::TopoShapePy::Type), &pObjBaseShape, &resolution)) {
        return nullptr;
    }
    CAMSim* sim = getCAMSimPtr();
    const Part::TopoShape& baseShape
        = static_cast<Part::TopoShapePy*>(pObjBaseShape)->getTopoShapePtr()->getShape();
    sim->SetBaseShape(baseShape, resolution);

    Py_IncRef(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::AddCommand(PyObject* args)
{
    PyObject* pObjCmd;
    if (!PyArg_ParseTuple(args, "O!", &(Path::CommandPy::Type), &pObjCmd)) {
        return nullptr;
    }
    CAMSim* sim = getCAMSimPtr();
    Path::Command* cmd = static_cast<Path::CommandPy*>(pObjCmd)->getCommandPtr();
    sim->AddCommand(cmd);

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::AddOperation(PyObject* args)
{
    const char* name;
    PyObject* pyColor = Py_None;
    if (!PyArg_ParseTuple(args, "s|O", &name, &pyColor)) {
        return nullptr;
    }
    float color[3];
    const bool hasColor = pyColor != Py_None;
    if (hasColor) {
        if (!PySequence_Check(pyColor) || PySequence_Size(pyColor) < 3) {
            PyErr_SetString(PyExc_TypeError, "color must be a sequence (r, g, b) or None");
            return nullptr;
        }
        for (int k = 0; k < 3; k++) {
            PyObject* item = PySequence_GetItem(pyColor, k);
            color[k] = (float)PyFloat_AsDouble(item);
            Py_XDECREF(item);
        }
        if (PyErr_Occurred()) {
            return nullptr;
        }
    }
    getCAMSimPtr()->AddOperation(name, hasColor ? color : nullptr);

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::SetFrameTiming(PyObject* args)
{
    PyObject* enabled;
    if (!PyArg_ParseTuple(args, "O!", &PyBool_Type, &enabled)) {
        return nullptr;
    }
    getCAMSimPtr()->SetFrameTiming(PyObject_IsTrue(enabled));

    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::TakeFrameTimes()
{
    const std::vector<double> times = getCAMSimPtr()->TakeFrameTimes();
    Py::List list;
    for (double time : times) {
        list.append(Py::Float(time));
    }
    return Py::new_reference_to(list);
}

PyObject* CAMSimPy::IsRenderPending()
{
    return Py::new_reference_to(Py::Boolean(getCAMSimPtr()->IsRenderPending()));
}

PyObject* CAMSimPy::GoToOperation(PyObject* args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index)) {
        return nullptr;
    }
    getCAMSimPtr()->GoToOperation(index);
    Py_INCREF(Py_None);
    return Py_None;
}

// A Python callable as a handler of an int, called with the interpreter locked; null for None
static bool IntHandler(PyObject* callback, std::function<void(int)>& handler)
{
    if (callback == Py_None) {
        handler = nullptr;
        return true;
    }
    if (!PyCallable_Check(callback)) {
        PyErr_SetString(PyExc_TypeError, "callback must be callable or None");
        return false;
    }
    // the callable is kept and released with the Python interpreter locked
    Py_INCREF(callback);
    std::shared_ptr<PyObject> held(callback, [](PyObject* object) {
        Base::PyGILStateLocker lock;
        Py_DECREF(object);
    });
    handler = [held](int value) {
        Base::PyGILStateLocker lock;
        PyObject* result = PyObject_CallFunction(held.get(), "i", value);
        if (!result) {
            Base::PyException exception;
            exception.reportException();
            return;
        }
        Py_DECREF(result);
    };
    return true;
}

PyObject* CAMSimPy::SetOperationPickedCallback(PyObject* args)
{
    PyObject* callback;
    std::function<void(int)> handler;
    if (!PyArg_ParseTuple(args, "O", &callback) || !IntHandler(callback, handler)) {
        return nullptr;
    }
    getCAMSimPtr()->SetOperationPickedHandler(handler);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::SetCommandChangedCallback(PyObject* args)
{
    PyObject* callback;
    std::function<void(int)> handler;
    if (!PyArg_ParseTuple(args, "O", &callback) || !IntHandler(callback, handler)) {
        return nullptr;
    }
    getCAMSimPtr()->SetCommandChangedHandler(handler);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::SetEventsChangedCallback(PyObject* args)
{
    PyObject* callback;
    std::function<void(int)> handler;
    if (!PyArg_ParseTuple(args, "O", &callback) || !IntHandler(callback, handler)) {
        return nullptr;
    }
    getCAMSimPtr()->SetEventsChangedHandler(handler);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::GoToEvent(PyObject* args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index)) {
        return nullptr;
    }
    getCAMSimPtr()->GoToEvent(index);
    Py_INCREF(Py_None);
    return Py_None;
}

// The names of the kinds of events
static const char* EventKindName(Material::Event::Kind kind)
{
    switch (kind) {
        case Material::Event::RapidIntoStock:
            return "rapid";
        case Material::Event::ShankContact:
            return "shank";
        case Material::Event::Gouge:
            return "gouge";
        case Material::Event::MaterialLeft:
            return "left";
    }
    return "";
}

PyObject* CAMSimPy::GetEvents()
{
    Py::List list;
    for (const SimEvent& e : getCAMSimPtr()->GetEvents()) {
        Py::Dict d;
        d.setItem("kind", Py::String(EventKindName(e.event.kind)));
        d.setItem("step", Py::Long(e.step));
        d.setItem("command", Py::Long(e.command));
        d.setItem("operation", Py::Long(e.event.operation));
        d.setItem("volume", Py::Float(e.event.volume));
        d.setItem("depth", Py::Float(e.event.depth));
        d.setItem("count", Py::Long(e.event.count));
        d.setItem(
            "box",
            Py::TupleN(
                Py::TupleN(Py::Float(e.event.lo[0]), Py::Float(e.event.lo[1]), Py::Float(e.event.lo[2])),
                Py::TupleN(Py::Float(e.event.hi[0]), Py::Float(e.event.hi[1]), Py::Float(e.event.hi[2]))
            )
        );
        d.setItem(
            "position",
            Py::TupleN(
                Py::Float(e.event.position[0]),
                Py::Float(e.event.position[1]),
                Py::Float(e.event.position[2])
            )
        );
        list.append(d);
    }
    return Py::new_reference_to(list);
}

PyObject* CAMSimPy::GetStockMesh()
{
    return new Mesh::MeshPy(getCAMSimPtr()->GetStockMesh());
}

PyObject* CAMSimPy::GetTimes()
{
    Py::Dict d;
    CAMSim* sim = getCAMSimPtr();
    d.setItem("total", Py::Float(sim->GetTotalSeconds()));
    Py::List ops;
    for (double s : sim->GetOperationSeconds()) {
        ops.append(Py::Float(s));
    }
    d.setItem("operations", ops);
    return Py::new_reference_to(d);
}

PyObject* CAMSimPy::GoToCommand(PyObject* args)
{
    int index;
    if (!PyArg_ParseTuple(args, "i", &index)) {
        return nullptr;
    }
    getCAMSimPtr()->GoToCommand(index);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::StepCommand(PyObject* args)
{
    int direction;
    if (!PyArg_ParseTuple(args, "i", &direction)) {
        return nullptr;
    }
    getCAMSimPtr()->StepCommand(direction);
    Py_INCREF(Py_None);
    return Py_None;
}

PyObject* CAMSimPy::GetRendererInfo()
{
    Py::Dict dict;
    for (const auto& [key, value] : getCAMSimPtr()->GetRendererInfo()) {
        dict.setItem(key, Py::String(value));
    }
    return Py::new_reference_to(dict);
}

PyObject* CAMSimPy::getCustomAttributes(const char* /*attr*/) const
{
    return nullptr;
}

int CAMSimPy::setCustomAttributes(const char* /*attr*/, PyObject* /*obj*/)
{
    return 0;
}

}  // namespace CAMSimulator
