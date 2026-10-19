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
    static const std::array<const char*, 5>
        kwlist {"shape", "toolnumber", "diameter", "resolution", nullptr};
    PyObject* pObjToolShape;
    int toolNumber;
    float resolution;
    float diameter;
    if (!Base::Wrapped_ParseTupleAndKeywords(
            args,
            kwds,
            "Oiff",
            kwlist,
            &pObjToolShape,
            &toolNumber,
            &diameter,
            &resolution
        )) {
        return nullptr;
    }
    // The tool shape is defined by a list of 2d points that represents the tool revolving profile
    Py_ssize_t num_floats = PyList_Size(pObjToolShape);
    std::vector<float> toolProfile;
    for (Py_ssize_t i = 0; i < num_floats; ++i) {
        PyObject* item = PyList_GetItem(pObjToolShape, i);
        toolProfile.push_back(static_cast<float>(PyFloat_AsDouble(item)));
    }

    CAMSim* sim = getCAMSimPtr();
    sim->addTool(toolProfile, toolNumber, diameter, resolution);

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

PyObject* CAMSimPy::SetOperationPickedCallback(PyObject* args)
{
    PyObject* callback;
    if (!PyArg_ParseTuple(args, "O", &callback)) {
        return nullptr;
    }
    if (callback == Py_None) {
        getCAMSimPtr()->SetOperationPickedHandler(nullptr);
        Py_INCREF(Py_None);
        return Py_None;
    }
    if (!PyCallable_Check(callback)) {
        PyErr_SetString(PyExc_TypeError, "callback must be callable or None");
        return nullptr;
    }
    // the callable is kept and released with the Python interpreter locked
    Py_INCREF(callback);
    std::shared_ptr<PyObject> held(callback, [](PyObject* object) {
        Base::PyGILStateLocker lock;
        Py_DECREF(object);
    });
    getCAMSimPtr()->SetOperationPickedHandler([held](int operation) {
        Base::PyGILStateLocker lock;
        PyObject* result = PyObject_CallFunction(held.get(), "i", operation);
        if (!result) {
            Base::PyException exception;
            exception.reportException();
            return;
        }
        Py_DECREF(result);
    });
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
