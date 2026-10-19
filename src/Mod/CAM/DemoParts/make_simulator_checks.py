# SPDX-License-Identifier: LGPL-2.1-or-later
# SPDX-FileNotice: Part of the FreeCAD project.

################################################################################
#                                                                              #
#   FreeCAD is free software: you can redistribute it and/or modify            #
#   it under the terms of the GNU Lesser General Public License as             #
#   published by the Free Software Foundation, either version 2.1              #
#   of the License, or (at your option) any later version.                     #
#                                                                              #
#   FreeCAD is distributed in the hope that it will be useful,                 #
#   but WITHOUT ANY WARRANTY; without even the implied warranty                #
#   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                    #
#   See the GNU Lesser General Public License for more details.                #
#                                                                              #
#   You should have received a copy of the GNU Lesser General Public           #
#   License along with FreeCAD. If not, see https://www.gnu.org/licenses       #
#                                                                              #
################################################################################

"""Generate simulator_checks.fcstd.

A demo job for the checks of the CAM simulator: a 130 x 90 x 40 mm block whose operations cut
what the model asks for, except where they deliberately don't. Simulate it (CAM_SimulatorGL) to see
what the simulator finds and shows.

Cut as the model has it, with tools whose profiles have arcs, followed exactly:

    pocket P1     bull nose D20 R5: the R5 fillets between the floor and the walls
    dish          rough with the bull nose, finish with a ball end D16
    scoop         a ball end D10 along an arc in the XZ plane (G18, a Custom operation)
    holes         drill cycles, peck drilling

Mistakes the simulator should find:

    slot          18 mm deep with an end mill of 10 mm flute length: its shank rubs the walls
    pocket P2     roughed with 0.5 mm left on the walls and 0.3 mm on the floor, never finished
    gouge         a Custom operation that cuts 1 mm into the top of the model
    rapid         a Custom operation that rapids into the stock (G0)

The tool controllers have their feeds and rapid rates, for the cycle time.

Run in the FreeCAD GUI so the document gets the view providers of the CAM objects. Headless (on
X11, without a Wayland session):

    WAYLAND_DISPLAY= QT_QPA_PLATFORM=xcb xvfb-run -a FreeCAD make_simulator_checks.py

Environment variables:
    CAM_DEMO_OUT   directory to write to, default: the directory of this script
    CAM_DEMO_EXIT  if set, quit the GUI after saving
"""

import importlib
import math
import os
import sys
import traceback

import FreeCAD
import Part

import Path.Main.Job as PathJob
import Path.Tool.Controller as PathToolController
from Path.Tool.toolbit import ToolBit

V = FreeCAD.Vector

OUT_DIR = os.environ.get("CAM_DEMO_OUT") or os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(OUT_DIR, "simulator_checks.fcstd")


# --------------------------------------------------------------------------- part geometry

L, W, H = 130.0, 90.0, 40.0

# pocket P1, floor fillets of the bull nose corner
P1 = (8.0, 8.0, 58.0, 42.0)  # x0, y0, x1, y1
P1_DEPTH, P1_CORNER, P1_FILLET = 12.0, 10.0, 5.0

DISH_C, DISH_R, DISH_DEPTH = V(95, 26, 0), 30.0, 8.0

# pocket P2, left unfinished
P2 = (8.0, 50.0, 40.0, 64.0)
P2_DEPTH, P2_CORNER = 5.0, 4.0

HOLES = [V(48, 57, 0), V(58, 57, 0)]
HOLE_D, HOLE_DEPTH = 6.0, 10.0

# slot deeper than the flutes of its end mill
SLOT = (70.0, 54.0, 120.0, 60.0)
SLOT_DEPTH = 18.0

# scoop: the tip of a ball end along an arc in the XZ plane, 4 mm deep in its middle
SCOOP_Y, SCOOP_X0, SCOOP_X1, SCOOP_DEPTH, SCOOP_BALL = 76.0, 10.0, 56.0, 4.0, 5.0
SCOOP_R = ((SCOOP_X1 - SCOOP_X0) ** 2 / 4 + SCOOP_DEPTH**2) / (2 * SCOOP_DEPTH)
SCOOP_CZ = H - SCOOP_DEPTH + SCOOP_R  # the center of the arc of the tip

GOUGE_Y, GOUGE_X0, GOUGE_X1, GOUGE_DEPTH = 84.0, 70.0, 90.0, 1.0
RAPID_Y, RAPID_X0, RAPID_X1, RAPID_DEPTH = 84.0, 100.0, 120.0, 2.0


def rounded_rect(x0, y0, x1, y1, r, z, h):
    box = Part.makeBox(x1 - x0, y1 - y0, h, V(x0, y0, z))
    edges = [e for e in box.Edges if abs(e.tangentAt(0).z) > 0.99]
    return box.makeFillet(r, edges)


def make_part_shape():
    block = Part.makeBox(L, W, H)

    # P1: walls with the corner radius, the floor joined to them by the fillet of the bull nose
    x0, y0, x1, y1 = P1
    p1 = rounded_rect(x0, y0, x1, y1, P1_CORNER, H - P1_DEPTH, P1_DEPTH + 1)
    floor = [e for e in p1.Edges if abs(e.BoundBox.ZMax - (H - P1_DEPTH)) < 1e-6]
    block = block.cut(p1.makeFillet(P1_FILLET, floor))

    block = block.cut(Part.makeSphere(DISH_R, V(DISH_C.x, DISH_C.y, H + DISH_R - DISH_DEPTH)))

    x0, y0, x1, y1 = P2
    block = block.cut(rounded_rect(x0, y0, x1, y1, P2_CORNER, H - P2_DEPTH, P2_DEPTH + 1))

    for c in HOLES:
        block = block.cut(Part.makeCylinder(HOLE_D / 2, HOLE_DEPTH, V(c.x, c.y, H - HOLE_DEPTH)))

    x0, y0, x1, y1 = SLOT
    slot = Part.makeBox(x1 - x0, y1 - y0, SLOT_DEPTH + 1, V(x0, y0, H - SLOT_DEPTH))
    block = block.cut(slot)

    # the ball center follows the arc of the tip, raised by the ball radius: a torus around Y
    center = V((SCOOP_X0 + SCOOP_X1) / 2, SCOOP_Y, SCOOP_CZ + SCOOP_BALL)
    block = block.cut(Part.makeTorus(SCOOP_R, SCOOP_BALL, center, V(0, 1, 0)))

    return block.removeSplitter()


# --------------------------------------------------------------------------- tools

# (number, label, shape, shape type, parameters, rpm, horizontal feed, vertical feed, rapid),
# speeds and feeds for carbide in aluminium, mm/min
# fmt: off
TOOLS = [
    (1, "T1 Bullnose D20 R5", "bullnose.fcstd", "Bullnose",
     {"Diameter": "20 mm", "CornerRadius": "5 mm", "CuttingEdgeHeight": "25 mm",
      "Length": "70 mm", "ShankDiameter": "20 mm", "Flutes": 3},
     6000, 1800, 500, 8000),
    (2, "T2 Ball end D16", "ballend.fcstd", "Ballend",
     {"Diameter": "16 mm", "CuttingEdgeHeight": "20 mm", "Length": "70 mm",
      "ShankDiameter": "16 mm", "Flutes": 2},
     7500, 2000, 500, 8000),
    (3, "T3 Ball end D10", "ballend.fcstd", "Ballend",
     {"Diameter": "10 mm", "CuttingEdgeHeight": "15 mm", "Length": "60 mm",
      "ShankDiameter": "10 mm", "Flutes": 2},
     9000, 1200, 400, 8000),
    (4, "T4 Endmill D8 3FL", "endmill.fcstd", "Endmill",
     {"Diameter": "8 mm", "CuttingEdgeHeight": "20 mm", "Length": "60 mm",
      "ShankDiameter": "8 mm", "Flutes": 3},
     10000, 1500, 400, 8000),
    (5, "T5 Endmill D6 short flutes", "endmill.fcstd", "Endmill",
     {"Diameter": "6 mm", "CuttingEdgeHeight": "10 mm", "Length": "50 mm",
      "ShankDiameter": "6 mm", "Flutes": 3},
     11000, 900, 300, 8000),
    (6, "T6 Drill D6", "drill.fcstd", "Drill",
     {"Diameter": "6 mm", "TipAngle": "118 deg", "Length": "70 mm", "Flutes": 2},
     6000, 600, 600, 8000),
]
# fmt: on


def make_tool_controllers(doc, job):
    tcs = {}
    for number, label, shape, shape_type, params, rpm, hfeed, vfeed, rapid in TOOLS:
        bit = ToolBit.from_dict(
            {
                "name": label,
                "shape": shape,
                "shape-type": shape_type,
                "parameter": dict(params),
                "attribute": {},
            }
        )
        tool = bit.attach_to_doc(doc=doc)
        tool.Label = label
        tc = PathToolController.Create("TC: " + label, tool=tool, toolNumber=number)
        tc.SpindleSpeed = float(rpm)
        tc.SpindleDir = "Forward"
        tc.HorizFeed = "%f mm/min" % hfeed
        tc.VertFeed = "%f mm/min" % vfeed
        tc.HorizRapid = "%f mm/min" % rapid
        tc.VertRapid = "%f mm/min" % (rapid / 2)
        job.Proxy.addToolController(tc)
        tcs[number] = tc
    return tcs


# --------------------------------------------------------------------------- features


def planar_face(shape, normal, point, tol=1e-3):
    """Name of the planar face with this outward normal that contains point."""
    normal = V(normal).normalize()
    vertex = Part.Vertex(point)
    for i, f in enumerate(shape.Faces):
        if f.Surface.TypeId != "Part::GeomPlane":
            continue
        n = f.normalAt(*f.Surface.parameter(f.CenterOfMass))
        if (n - normal).Length > 1e-4:
            continue
        if f.distToShape(vertex)[0] < tol:
            return "Face%d" % (i + 1)
    raise ValueError("no planar face %s through %s" % (normal, point))


def faces_of_type(shape, type_id):
    return ["Face%d" % (i + 1) for i, f in enumerate(shape.Faces) if f.Surface.TypeId == type_id]


def cylinder_faces(shape, centers, radius):
    found = []
    for c in centers:
        for i, f in enumerate(shape.Faces):
            s = f.Surface
            if s.TypeId != "Part::GeomCylinder" or abs(s.Radius - radius) > 1e-3:
                continue
            offset = s.Center - c
            if abs(offset.x) < 1e-3 and abs(offset.y) < 1e-3:
                found.append("Face%d" % (i + 1))
                break
        else:
            raise ValueError("no cylinder r=%s at %s" % (radius, c))
    return found


def gcode(*lines):
    return list(lines)


def f(v):
    return ("%.4f" % v).rstrip("0").rstrip(".")


# --------------------------------------------------------------------------- operations

DEPTHS = ("StartDepth", "FinalDepth", "StepDown")


class Builder:
    def __init__(self, doc, job):
        self.doc = doc
        self.job = job
        self.ops = []

    def op(self, module, label, tc, base=None, **props):
        """Add an operation like the GUI command does, then set its properties. A depth given
        here replaces the expression that would compute it. The tool controller is selected
        first: an operation asks for one when the job has several and none is selected."""
        if FreeCAD.GuiUp:
            import FreeCADGui

            FreeCADGui.Selection.clearSelection()
            if tc is not None:
                FreeCADGui.Selection.addSelection(tc)
        obj = importlib.import_module("Path.Op." + module).Create(module, parentJob=self.job)
        if FreeCAD.GuiUp:
            FreeCADGui.Selection.clearSelection()
        if FreeCAD.GuiUp:
            import Path.Op.Gui.Base as PathOpGui

            gui = importlib.import_module("Path.Op.Gui." + module)
            obj.ViewObject.Proxy = PathOpGui.ViewProvider(obj.ViewObject, gui.Command.res)
        obj.Label = label
        if tc is not None:
            obj.ToolController = tc
        if hasattr(obj, "CoolantMode"):
            obj.CoolantMode = "Flood"
        if base is not None:
            obj.Base = base
        for name, value in props.items():
            if name in DEPTHS:
                obj.setExpression(name, None)
            setattr(obj, name, value)
        self.doc.recompute()
        self.ops.append(obj)
        return obj

    def report(self):
        """Print one line per operation, so an operation that produced no path stands out."""
        ok = True
        for obj in self.ops:
            cmds = obj.Path.Commands
            cutting = [c for c in cmds if c.Name[0] == "G" and c.Name not in ("G0", "G00")]
            status = obj.getStatusString()
            good = bool(cutting) and "rror" not in status
            ok = ok and good
            print("%s %-44s %6d commands %s" % ("ok " if good else "BAD", obj.Label, len(cmds), status))
        return ok


def build():
    doc = FreeCAD.newDocument("simulator_checks")

    part = doc.addObject("Part::Feature", "CheckBlock")
    part.Label = "Simulator check block"
    part.Shape = make_part_shape()
    doc.recompute()

    if FreeCAD.GuiUp:
        import FreeCADGui
        import Path.Main.Gui.Job as PathJobGui

        part.ViewObject.Visibility = False
        FreeCADGui.activateWorkbench("CAMWorkbench")
        job = PathJobGui.Create([part], None, openTaskPanel=False)
    else:
        job = PathJob.Create("Job", [part], None)

    job.Label = "Simulator checks"
    job.Description = (
        "Demo of the checks of the CAM simulator: exact tool arcs, a shank rubbing the walls of a "
        "deep slot, material left, a gouge and a rapid into the stock. Do not machine."
    )
    job.GeometryTolerance = 0.01
    stock = job.Stock
    stock.ExtXneg = stock.ExtXpos = 0.0
    stock.ExtYneg = stock.ExtYpos = 0.0
    stock.ExtZpos = 0.0
    stock.ExtZneg = 0.0
    job.SetupSheet.SafeHeightOffset = 3.0
    job.SetupSheet.ClearanceHeightOffset = 5.0

    for tc in list(job.Tools.Group):  # the default tool controller
        if tc.Tool:
            doc.removeObject(tc.Tool.Name)
        doc.removeObject(tc.Name)
    T = make_tool_controllers(doc, job)
    doc.recompute()

    model = job.Model.Group[0]
    shape = model.Shape

    p1_floor = planar_face(shape, V(0, 0, 1), V(P1[0] + 15, P1[1] + 15, H - P1_DEPTH))
    dish = faces_of_type(shape, "Part::GeomSphere")
    p2_floor = planar_face(shape, V(0, 0, 1), V(P2[0] + 6, P2[1] + 6, H - P2_DEPTH))
    slot_floor = planar_face(shape, V(0, 0, 1), V(SLOT[0] + 5, SLOT[1] + 3, H - SLOT_DEPTH))
    holes = cylinder_faces(shape, HOLES, HOLE_D / 2)

    clear = H + 5
    scoop_i = (SCOOP_X1 - SCOOP_X0) / 2
    scoop_k = SCOOP_CZ - H

    b = Builder(doc, job)
    # fmt: off
    # the floor face ends where the fillets start: the pocket 5 mm larger reaches the walls
    b.op("PocketShape", "010 Pocket P1 - bull nose R5 fillets", T[1], base=[(model, [p1_floor])],
         StepOver=40, StepDown=4.0, ExtraOffset=-P1_FILLET, ClearingPattern="Offset")
    b.op("Pocket", "020 Rough dish - bull nose", T[1], base=[(model, dish)], StepOver=40,
         StepDown=2.0, ExtraOffset=0.5)
    b.op("Surface", "030 Finish dish - ball end D16", T[2], base=[(model, dish)],
         CutPattern="Line", StepOver=10, BoundBox="BaseBoundBox")
    b.op("Custom", "040 Scoop - ball end along a G18 arc", T[3], Source="Text", Gcode=gcode(
        "(the tip along an arc in the XZ plane, 4 mm deep in the middle)",
        "G0 X%s Y%s Z%s" % (f(SCOOP_X0), f(SCOOP_Y), f(clear)),
        "G1 Z%s" % f(H),
        "G18",
        "G2 X%s Z%s I%s K%s" % (f(SCOOP_X1), f(H), f(scoop_i), f(scoop_k)),
        "G17",
        "G0 Z%s" % f(clear),
    ))
    b.op("Drilling", "050 Peck drill holes", T[6], base=[(model, holes)], PeckEnabled=True,
         PeckDepth=3.0, ExtraOffset="None", FinalDepth=H - HOLE_DEPTH)
    # mistakes
    b.op("Slot", "060 Slot - shank rubs, flutes only 10 mm", T[5], base=[(model, [slot_floor])],
         StepDown=6.0)
    b.op("Adaptive", "070 Rough P2 - never finished", T[4], base=[(model, [p2_floor])],
         Side="Inside", OperationType="Clearing", StepOverPercent=20.0, StockToLeave=0.5,
         ZStockToLeave=0.3, StepDown=5.0)
    b.op("Custom", "080 Gouge - 1 mm into the model", T[5], Source="Text", Gcode=gcode(
        "G0 X%s Y%s Z%s" % (f(GOUGE_X0), f(GOUGE_Y), f(clear)),
        "G1 Z%s" % f(H - GOUGE_DEPTH),
        "G1 X%s" % f(GOUGE_X1),
        "G0 Z%s" % f(clear),
    ))
    b.op("Custom", "090 Rapid into the stock", T[5], Source="Text", Gcode=gcode(
        "G0 X%s Y%s Z%s" % (f(RAPID_X0), f(RAPID_Y), f(clear)),
        "G0 Z%s" % f(H - RAPID_DEPTH),
        "G0 X%s" % f(RAPID_X1),
        "G0 Z%s" % f(clear),
    ))
    # fmt: on

    doc.recompute()
    ok = b.report()
    if os.path.exists(OUT):
        os.remove(OUT)  # replaced, not backed up next to it
    doc.saveAs(OUT)
    print("saved %s" % OUT)
    return ok


try:
    build()
except Exception:
    traceback.print_exc()
if FreeCAD.GuiUp and os.environ.get("CAM_DEMO_EXIT"):
    sys.stdout.flush()
    os._exit(0)
