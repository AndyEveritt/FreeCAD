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

"""Generate manifold_block_3plus2.fcstd and Demo_5-Axis_AC_Trunnion.fcm.

A demo job that uses every tool shape and every 3-axis operation (except Rotary Surface), set up
like a job for a real machine: a 140 x 90 x 45 mm 6061 block machined done-in-one on a 5-axis
trunnion, with 3+2 work planes for the front, the right side and a 45 degree back chamfer.

    Top (+Z):   pocket with an island boss, spherical dish, M16x1.5 bore, 4x M8 tapped holes,
                2x 6H7 dowel holes, 2 ramped V flutes
    Back:       45 degree chamfer, part number V-carved into it
    Front (-Y): T-slot through in X, 2x M8 tapped holes, serial number engraved
    Right (+X): dovetail slot through in Y

The undercut tools (slitting saw, dovetail) enter and leave the slots outside the part, they can
not plunge or retract through the narrow neck of a slot.

Run in the FreeCAD GUI so the document gets the view providers of the CAM objects; FreeCADCmd
works too, but the operations then have no task panels when the file is opened. Headless:

    xvfb-run -a FreeCAD make_manifold_block_3plus2.py

Environment variables:
    CAM_DEMO_OUT   directory to write to, default: the directory of this script
    CAM_DEMO_EXIT  if set, quit the GUI after saving

The posted program needs the machine: copy Demo_5-Axis_AC_Trunnion.fcm into the Machines folder
of the CAM assets.
"""

import importlib
import math
import os
import sys
import traceback

import FreeCAD
import Part

import Path.Main.Job as PathJob
import Path.Main.Workplane as PathWorkplane
import Path.Tool.Controller as PathToolController
from Path.Tool.toolbit import ToolBit
from Machine.models.machine import (
    AxisRole,
    Machine,
    MachineFactory,
    RotaryAxis,
    RotationStrategy,
    Toolhead,
    ToolheadType,
)

V = FreeCAD.Vector

OUT_DIR = os.environ.get("CAM_DEMO_OUT") or os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(OUT_DIR, "manifold_block_3plus2.fcstd")
FONT = os.path.join(FreeCAD.getResourceDir(), "examples", "osifont-lgpl3fe.ttf")

MACHINE_NAME = "Demo 5-Axis AC Trunnion"
MACHINE_FILE = "Demo_5-Axis_AC_Trunnion.fcm"


# --------------------------------------------------------------------------- part geometry

L, W, H = 140.0, 90.0, 45.0

# back chamfer: from (Y=75, Z=45) down to (Y=90, Z=30)
CHAMFER_Y, CHAMFER_Z = 75.0, 30.0

POCKET = (12.0, 15.0, 62.0, 62.0)  # x0, y0, x1, y1
POCKET_DEPTH = 15.0
POCKET_R = 6.0
BOSS_C, BOSS_D = V(37, 38.5, 0), 20.0

DISH_C, DISH_R, DISH_DEPTH = V(108, 37, 0), 35.0, 8.0

BORE_C, BORE_D, BORE_DEPTH = V(75, 37, 0), 14.4, 20.0  # M16x1.5 minor diameter

M8_TOP = [V(7, 16, 0), V(133, 16, 0), V(7, 68, 0), V(133, 68, 0)]
M8_DRILL, M8_DEPTH = 6.8, 12.0
DOWELS = [V(75, 16, 0), V(75, 66, 0)]
DOWEL_D, DOWEL_DEPTH = 6.0, 12.0

FLUTES_Y = [66.0, 71.0]
FLUTE_X0, FLUTE_X1, FLUTE_DEPTH = 20.0, 58.0, 2.0

# front T-slot, through in X so the slitting saw enters and leaves outside the part
TSLOT_ZC, TSLOT_NECK, TSLOT_NECK_DEPTH = 33.0, 6.0, 6.0
SAW_D, SAW_T = 16.0, 3.0
M8_FRONT = [V(20, 0, 8), V(120, 0, 8)]
M8_FRONT_DEPTH = 16.0

# right dovetail, through in Y
DOVE_ZC, DOVE_DEPTH, DOVE_ANGLE, DOVE_BOTTOM = 14.0, 6.0, 60.0, 19.0
DOVE_TOP = DOVE_BOTTOM - 2 * DOVE_DEPTH / math.tan(math.radians(DOVE_ANGLE))


def rounded_rect(x0, y0, x1, y1, r, z, h):
    box = Part.makeBox(x1 - x0, y1 - y0, h, V(x0, y0, z))
    edges = [e for e in box.Edges if abs(e.tangentAt(0).z) > 0.99]
    return box.makeFillet(r, edges)


def cyl(d, depth, c, top):
    return Part.makeCylinder(d / 2, depth, V(c.x, c.y, top - depth))


def make_part_shape():
    block = Part.makeBox(L, W, H)

    # back chamfer
    tri = Part.makePolygon([V(0, CHAMFER_Y, H), V(0, W, H), V(0, W, CHAMFER_Z), V(0, CHAMFER_Y, H)])
    block = block.cut(Part.Face(tri).extrude(V(L, 0, 0)))

    # pocket with island
    x0, y0, x1, y1 = POCKET
    pocket = rounded_rect(x0, y0, x1, y1, POCKET_R, H - POCKET_DEPTH, POCKET_DEPTH + 1)
    pocket = pocket.cut(cyl(BOSS_D, POCKET_DEPTH + 1, BOSS_C, H + 1))
    block = block.cut(pocket)

    # spherical dish
    block = block.cut(Part.makeSphere(DISH_R, V(DISH_C.x, DISH_C.y, H + DISH_R - DISH_DEPTH)))

    # holes
    holes = [cyl(BORE_D, BORE_DEPTH, BORE_C, H)]
    holes += [cyl(M8_DRILL, M8_DEPTH, c, H) for c in M8_TOP]
    holes += [cyl(DOWEL_D, DOWEL_DEPTH, c, H) for c in DOWELS]
    for c in M8_FRONT:
        holes.append(Part.makeCylinder(M8_DRILL / 2, M8_FRONT_DEPTH, V(c.x, 0, c.z), V(0, 1, 0)))
    for hole in holes:
        block = block.cut(hole)

    # ramped 90 degree V flutes: zero depth at FLUTE_X0, FLUTE_DEPTH deep at FLUTE_X1
    slope = math.atan2(FLUTE_DEPTH, FLUTE_X1 - FLUTE_X0)
    for y in FLUTES_Y:
        h = 10.0
        tri = Part.makePolygon([V(0, y - h, h), V(0, y + h, h), V(0, y, 0), V(0, y - h, h)])
        prism = Part.Face(tri).extrude(V(FLUTE_X1 - FLUTE_X0 + 20, 0, 0))
        prism.translate(V(FLUTE_X0 - 10, 0, 0))
        prism.rotate(V(FLUTE_X0, y, 0), V(0, 1, 0), math.degrees(slope))
        prism.translate(V(0, 0, H))
        limit = Part.makeBox(FLUTE_X1 - FLUTE_X0, 30, 20, V(FLUTE_X0, y - 15, H - 10))
        block = block.cut(prism.common(limit))

    # front T-slot: neck plus saw undercut
    r = SAW_D / 2
    neck = Part.makeBox(
        L + 20, TSLOT_NECK_DEPTH + 1, TSLOT_NECK, V(-10, -1, TSLOT_ZC - TSLOT_NECK / 2)
    )
    under = Part.makeBox(L + 20, SAW_T, SAW_D, V(-10, TSLOT_NECK_DEPTH, TSLOT_ZC - r))
    block = block.cut(neck.fuse(under))

    # right dovetail
    t, b, d = DOVE_TOP / 2, DOVE_BOTTOM / 2, DOVE_DEPTH
    run = 1 / math.tan(math.radians(DOVE_ANGLE))
    pts = [
        V(L + 1, -1, DOVE_ZC - t - run),
        V(L - d, -1, DOVE_ZC - b),
        V(L - d, -1, DOVE_ZC + b),
        V(L + 1, -1, DOVE_ZC + t + run),
    ]
    block = block.cut(Part.Face(Part.makePolygon(pts + [pts[0]])).extrude(V(0, W + 2, 0)))

    return block.removeSplitter()


# --------------------------------------------------------------------------- machine


def make_machine():
    m = Machine.create_3axis_config()
    m.name = MACHINE_NAME
    m.manufacturer = "FreeCAD demo"
    m.description = (
        "Generic table-table 5-axis VMC: C rotary table carried by an A trunnion "
        "(-120..+30 deg). Indexes for 3+2 machining with a Fanuc style tilted work plane "
        "(G68.2). 12000 rpm spindle, 24 pocket tool changer."
    )
    m.rotary_axes["C"] = RotaryAxis(
        name="C",
        rotation_vector=V(0, 0, 1),
        min_limit=-360,
        max_limit=360,
        role=AxisRole.TABLE_ROTARY,
        sequence=0,
    )
    m.rotary_axes["A"] = RotaryAxis(
        name="A",
        rotation_vector=V(1, 0, 0),
        min_limit=-120,
        max_limit=30,
        role=AxisRole.TABLE_ROTARY,
        parent="C",
        sequence=1,
    )
    m.kinematics.rotation_strategy = RotationStrategy.TWP
    m.toolheads = [
        Toolhead(
            name="Spindle",
            toolhead_type=ToolheadType.ROTARY,
            min_rpm=100,
            max_rpm=12000,
            max_power_kw=11.0,
            tool_change="automatic",
            coolant_flood=True,
            coolant_mist=True,
        )
    ]
    for name, travel in (("X", 650), ("Y", 520), ("Z", 460)):
        m.linear_axes[name].max_limit = travel
        m.linear_axes[name].max_velocity = 36000
    m.postprocessor_file_name = "fanuc"
    return m


# --------------------------------------------------------------------------- finding features


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


def cylinder_faces(shape, centers, radius, axis=V(0, 0, 1)):
    """Names of the cylindrical faces of radius whose axis passes through centers."""
    found = []
    for c in centers:
        for i, f in enumerate(shape.Faces):
            s = f.Surface
            if s.TypeId != "Part::GeomCylinder" or abs(s.Radius - radius) > 1e-3:
                continue
            if abs(abs(s.Axis.dot(axis)) - 1) > 1e-6:
                continue
            offset = s.Center - c
            if (offset - axis * offset.dot(axis)).Length < 1e-3:
                found.append("Face%d" % (i + 1))
                break
        else:
            raise ValueError("no cylinder r=%s at %s" % (radius, c))
    return found


def faces_of_type(shape, type_id):
    return ["Face%d" % (i + 1) for i, f in enumerate(shape.Faces) if f.Surface.TypeId == type_id]


def straight_edges(shape, predicate):
    """Names of the straight edges whose end points satisfy predicate(start, end)."""
    return [
        "Edge%d" % (i + 1)
        for i, e in enumerate(shape.Edges)
        if len(e.Vertexes) == 2 and predicate(e.Vertexes[0].Point, e.Vertexes[1].Point)
    ]


def boundary_edges(shape, face_name, predicate):
    """Names of the edges of face_name that satisfy predicate(edge)."""
    face = shape.getElement(face_name)
    return [
        "Edge%d" % (i + 1)
        for e in face.Edges
        for i, e2 in enumerate(shape.Edges)
        if e.isSame(e2) and predicate(e2)
    ]


# --------------------------------------------------------------------------- markings


def frame(origin, x_dir, y_dir):
    x = V(x_dir).normalize()
    y = V(y_dir).normalize()
    z = x.cross(y)
    m = FreeCAD.Matrix(x.x, y.x, z.x, origin.x, x.y, y.y, z.y, origin.y, x.z, y.z, z.z, origin.z)
    return FreeCAD.Placement(m)


def text_shape(text, size, placement):
    """Glyph faces of text, centred on placement. Stored as a plain shape so the document does
    not depend on the font file."""
    import Draft

    doc = FreeCAD.ActiveDocument
    ss = Draft.make_shape_string(text, FONT, size)
    doc.recompute()
    shape = ss.Shape.copy()
    doc.removeObject(ss.Name)
    bb = shape.BoundBox
    shape.translate(V(-(bb.XMin + bb.XMax) / 2, -(bb.YMin + bb.YMax) / 2, 0))
    shape.Placement = placement.multiply(shape.Placement)
    return shape


# --------------------------------------------------------------------------- tools

# (number, label, shape, shape type, parameters, rpm, horizontal feed, vertical feed),
# speeds and feeds for carbide in aluminium
# fmt: off
TOOLS = [
    (1, "T1 Face mill D50", "endmill.fcstd", "Endmill",
     {"Diameter": "50 mm", "CuttingEdgeHeight": "6 mm", "Length": "45 mm",
      "ShankDiameter": "22 mm", "Flutes": 5},
     5700, 2500, 400),
    (2, "T2 Endmill D10 3FL", "endmill.fcstd", "Endmill",
     {"Diameter": "10 mm", "CuttingEdgeHeight": "22 mm", "Length": "75 mm",
      "ShankDiameter": "10 mm", "Flutes": 3},
     9500, 1500, 500),
    (3, "T3 Endmill D6 3FL", "endmill.fcstd", "Endmill",
     {"Diameter": "6 mm", "CuttingEdgeHeight": "18 mm", "Length": "60 mm",
      "ShankDiameter": "6 mm", "Flutes": 3},
     11000, 1000, 300),
    (4, "T4 Bullnose D8 R1", "bullnose.fcstd", "Bullnose",
     {"Diameter": "8 mm", "CornerRadius": "1 mm", "CuttingEdgeHeight": "16 mm",
      "Length": "60 mm", "ShankDiameter": "8 mm", "Flutes": 3},
     10000, 1400, 400),
    (5, "T5 Ball end D6", "ballend.fcstd", "Ballend",
     {"Diameter": "6 mm", "CuttingEdgeHeight": "12 mm", "Length": "60 mm",
      "ShankDiameter": "6 mm", "Flutes": 2},
     11000, 1500, 400),
    (6, "T6 Tapered ball R0.5 3deg", "taperedballnose.fcstd", "TaperedBallNose",
     {"Diameter": "1 mm", "TaperAngle": "3 deg", "TaperDiameter": "4 mm",
      "CuttingEdgeHeight": "20 mm", "Length": "60 mm", "ShankDiameter": "6 mm", "Flutes": 2},
     12000, 1200, 300),
    (7, "T7 Chamfer mill D12 90deg", "chamfer.fcstd", "Chamfer",
     {"Diameter": "12 mm", "CuttingEdgeAngle": "90 deg", "TipDiameter": "0.5 mm",
      "CuttingEdgeHeight": "6 mm", "Length": "55 mm", "ShankDiameter": "12 mm", "Flutes": 4},
     8000, 800, 300),
    (8, "T8 Corner rounding R2", "radius.fcstd", "Radius",
     {"Diameter": "10 mm", "CuttingRadius": "2 mm", "TipDiameter": "6 mm",
      "CuttingEdgeHeight": "4 mm", "Length": "55 mm", "ShankDiameter": "10 mm", "Flutes": 3},
     8000, 800, 300),
    (9, "T9 V-bit 60deg D6", "v-bit.fcstd", "VBit",
     {"Diameter": "6 mm", "CuttingEdgeAngle": "60 deg", "TipDiameter": "0.1 mm",
      "CuttingEdgeHeight": "5 mm", "Length": "50 mm", "ShankDiameter": "6 mm", "Flutes": 2},
     12000, 600, 200),
    (10, "T10 V-bit 90deg D10", "v-bit.fcstd", "VBit",
     {"Diameter": "10 mm", "CuttingEdgeAngle": "90 deg", "TipDiameter": "0.1 mm",
      "CuttingEdgeHeight": "5 mm", "Length": "50 mm", "ShankDiameter": "10 mm", "Flutes": 2},
     10000, 800, 200),
    (11, "T11 Drill D6.8 - M8 tap drill", "drill.fcstd", "Drill",
     {"Diameter": "6.8 mm", "TipAngle": "140 deg", "Length": "75 mm", "Flutes": 2},
     6000, 700, 700),
    (12, "T12 Drill D5.8 - 6H7 pre-drill", "drill.fcstd", "Drill",
     {"Diameter": "5.8 mm", "TipAngle": "140 deg", "Length": "70 mm", "Flutes": 2},
     6500, 700, 700),
    (13, "T13 Reamer D6 H7", "reamer.fcstd", "Reamer",
     {"Diameter": "6 mm", "CuttingEdgeHeight": "20 mm", "Length": "75 mm",
      "ShankDiameter": "6 mm"},
     1600, 320, 320),
    (14, "T14 Tap M8x1.25", "tap.fcstd", "Tap",
     {"Diameter": "8 mm", "Pitch": "1.25 mm", "CuttingEdgeLength": "14 mm",
      "TipAngle": "90 deg", "Length": "70 mm", "ShankDiameter": "6.2 mm", "Flutes": 3},
     600, 750, 750),
    (15, "T15 Thread mill D10 P1.5", "thread-mill.fcstd", "ThreadMill",
     {"Diameter": "10 mm", "Crest": "0.19 mm", "NeckDiameter": "7 mm", "NeckLength": "25 mm",
      "Length": "65 mm", "ShankDiameter": "10 mm", "Flutes": 3},
     7000, 500, 200),
    (16, "T16 Dovetail D19 60deg", "dovetail.fcstd", "Dovetail",
     {"Diameter": "19 mm", "CuttingEdgeAngle": "60 deg", "CuttingEdgeHeight": "6.5 mm",
      "TipDiameter": "19 mm", "NeckDiameter": "8 mm", "NeckHeight": "10 mm",
      "Length": "60 mm", "ShankDiameter": "12 mm", "Flutes": 6},
     5000, 350, 200),
    (17, "T17 Slitting saw D16x3", "slittingsaw.fcstd", "SlittingSaw",
     {"Diameter": "16 mm", "BladeThickness": "3 mm", "CapDiameter": "5 mm",
      "CapHeight": "1 mm", "Length": "55 mm", "ShankDiameter": "5 mm", "Flutes": 8},
     4500, 300, 200),
    (18, "T18 Probe D2 ruby", "probe.fcstd", "Probe",
     {"Diameter": "2 mm", "ShaftDiameter": "2 mm", "Length": "50 mm"},
     0, 300, 200),
]
# fmt: on


def make_tool_controllers(doc, job):
    tcs = {}
    for number, label, shape, shape_type, params, rpm, hfeed, vfeed in TOOLS:
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
        tc.SpindleDir = "Forward" if rpm else "None"
        tc.HorizFeed = "%f mm/min" % hfeed
        tc.VertFeed = "%f mm/min" % vfeed
        job.Proxy.addToolController(tc)
        tcs[number] = tc
    return tcs


# --------------------------------------------------------------------------- operations

DEPTHS = ("StartDepth", "FinalDepth", "StepDown")


class Builder:
    def __init__(self, doc, job):
        self.doc = doc
        self.job = job
        self.ops = []

    def op(self, module, label, tc, workplane=None, base=None, coolant="Flood", **props):
        """Add an operation like the GUI command does, then set its properties. A depth given
        here replaces the expression that would compute it."""
        obj = importlib.import_module("Path.Op." + module).Create(module, parentJob=self.job)
        if FreeCAD.GuiUp:
            import Path.Op.Gui.Base as PathOpGui

            gui = importlib.import_module("Path.Op.Gui." + module)
            obj.ViewObject.Proxy = PathOpGui.ViewProvider(obj.ViewObject, gui.Command.res)
        obj.Label = label
        if tc is not None:
            obj.ToolController = tc
        if hasattr(obj, "CoolantMode"):
            obj.CoolantMode = coolant
        # always set: a new operation inherits the work plane of the previous one
        obj.Workplane = workplane
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
            good = obj.Label.startswith("145") or (cutting and "rror" not in status)
            ok = ok and good
            plane = obj.Workplane.Label if obj.Workplane else "-"
            print(
                "%s %-42s %-22s %6d commands %s"
                % ("ok " if good else "BAD", obj.Label, plane, len(cmds), status)
            )
        return ok


def build():
    MachineFactory.set_config_directory(OUT_DIR)
    MachineFactory.save_configuration(make_machine(), MACHINE_FILE)

    doc = FreeCAD.newDocument("manifold_block_3plus2")

    part = doc.addObject("Part::Feature", "ManifoldBlock")
    part.Label = "Manifold block"
    part.Shape = make_part_shape()

    # markings, machined but not modelled
    a = math.sqrt(0.5)
    pn = doc.addObject("Part::Feature", "PartNumber")
    pn.Label = "Part number marking"
    pn.Shape = text_shape("FC-CAM-3+2", 7.0, frame(V(70, 82.5, 37.5), V(-1, 0, 0), V(0, -a, a)))
    sn = doc.addObject("Part::Feature", "SerialNumber")
    sn.Label = "Serial number marking"
    sn.Shape = text_shape("SN 0001", 5.0, frame(V(70, 0, 6.0), V(1, 0, 0), V(0, 0, 1)))
    area = doc.addObject("Part::Feature", "ProbeArea")
    area.Label = "Probe area"
    area.Shape = Part.makeBox(120, 64, 10, V(10, 8, 40))
    doc.recompute()

    if FreeCAD.GuiUp:
        import FreeCADGui
        import Path.Main.Gui.Job as PathJobGui

        for obj in (part, pn, sn, area):
            obj.ViewObject.Visibility = False
        FreeCADGui.activateWorkbench("CAMWorkbench")
        job = PathJobGui.Create([part], None, openTaskPanel=False)
    else:
        job = PathJob.Create("Job", [part], None)

    job.Label = "OP10 Manifold block - 5 axis done in one"
    # One line: the post writes the description into the program header as a comment.
    job.Description = (
        "6061-T6 blank 142 x 92 x 56.5 mm held on 10 mm extra stock in a 5-axis dovetail vise. "
        "G54 X0 Y0 front left corner, Z0 part bottom. Probe stock top before running. "
        "Saw off and face bottom in OP20."
    )
    job.Machine = MACHINE_NAME
    job.PostProcessorOutputFile = "%d/O1001_manifold_block.nc"
    job.Fixtures = ["G54"]
    job.GeometryTolerance = 0.01
    stock = job.Stock
    stock.ExtXneg = stock.ExtXpos = 1.0
    stock.ExtYneg = stock.ExtYpos = 1.0
    stock.ExtZpos = 1.5
    stock.ExtZneg = 10.0  # held in the vise
    job.SetupSheet.SafeHeightOffset = 3.0
    job.SetupSheet.ClearanceHeightOffset = 10.0

    for tc in list(job.Tools.Group):  # the default tool controller
        if tc.Tool:
            doc.removeObject(tc.Tool.Name)
        doc.removeObject(tc.Name)
    T = make_tool_controllers(doc, job)
    doc.recompute()

    model = job.Model.Group[0]
    shape = model.Shape

    # 3+2 work planes, on the faces they machine
    wp_back = PathWorkplane.createWorkplane(
        job, model, planar_face(shape, V(0, 1, 1), V(70, 82.5, 37.5)), label="WP Back chamfer"
    )
    wp_front = PathWorkplane.createWorkplane(
        job, model, planar_face(shape, V(0, -1, 0), V(70, 0, 44)), label="WP Front"
    )
    wp_right = PathWorkplane.createWorkplane(
        job, model, planar_face(shape, V(1, 0, 0), V(L, 45, 40)), label="WP Right"
    )

    top = planar_face(shape, V(0, 0, 1), V(1, 1, H))
    pocket_floor = planar_face(shape, V(0, 0, 1), V(15, 20, H - POCKET_DEPTH))
    dish = faces_of_type(shape, "Part::GeomSphere")
    bore = cylinder_faces(shape, [BORE_C], BORE_D / 2)
    m8_top = cylinder_faces(shape, M8_TOP, M8_DRILL / 2)
    m8_front = cylinder_faces(shape, M8_FRONT, M8_DRILL / 2, axis=V(0, 1, 0))
    dowels = cylinder_faces(shape, DOWELS, DOWEL_D / 2)
    dove_floor = planar_face(shape, V(1, 0, 0), V(L - DOVE_DEPTH, 45, DOVE_ZC))
    neck_z = (TSLOT_ZC - TSLOT_NECK / 2, TSLOT_ZC + TSLOT_NECK / 2)
    tslot_neck = straight_edges(
        shape,
        lambda a, b: abs(a.y) < 1e-6
        and abs(b.y) < 1e-6
        and abs(a.z - b.z) < 1e-6
        and any(abs(a.z - z) < 1e-6 for z in neck_z),
    )
    flute_bottoms = straight_edges(
        shape,
        lambda a, b: abs(a.y - b.y) < 1e-6
        and any(abs(a.y - y) < 1e-6 for y in FLUTES_Y)
        and abs(a.x - b.x) > 30,
    )
    assert len(tslot_neck) == 2 and len(flute_bottoms) == 2, (tslot_neck, flute_bottoms)

    def front_or_side(e):
        a, b = e.Vertexes[0].Point, e.Vertexes[-1].Point
        if abs(a.y) < 1e-6 and abs(b.y) < 1e-6:
            return True
        return abs(a.x - b.x) < 1e-6 and (abs(a.x) < 1e-6 or abs(a.x - L) < 1e-6)

    def in_pocket(e):
        bb = e.BoundBox
        x0, y0, x1, y1 = POCKET
        inside_x = x0 - 1e-6 <= bb.XMin and bb.XMax <= x1 + 1e-6
        return inside_x and y0 - 1e-6 <= bb.YMin and bb.YMax <= y1 + 1e-6

    def bore_rim(e):
        return hasattr(e.Curve, "Radius") and abs(e.Curve.Radius - BORE_D / 2) < 1e-3

    outer_top_edges = boundary_edges(shape, top, front_or_side)
    rims = boundary_edges(shape, top, lambda e: in_pocket(e) or bore_rim(e))

    b = Builder(doc, job)
    # fmt: off
    # roughing
    b.op("Probe", "010 Probe stock top - surface map", T[18], coolant="None", BaseShape=area,
         PointCountX=4, PointCountY=3, OutputFileName="probe_top.txt")
    b.op("MillFacing", "020 Face top", T[1], StepOver=70)
    b.op("MillFacing", "030 Face back chamfer", T[1], workplane=wp_back, StepOver=70,
         StepDown=3.0)
    b.op("Adaptive", "040 Adaptive rough pocket", T[2], base=[(model, [pocket_floor])],
         Side="Inside", OperationType="Clearing", StepOverPercent=12.0, StockToLeave=0.3,
         ZStockToLeave=0.2, StepDown=7.5)
    b.op("Profile", "050 Profile outside", T[2], Side="Outside", Direction="CW",
         processHoles=False, processPerimeter=True, FinalDepth=0.0, StepDown=11.25)
    b.op("Helix", "060 Helix bore M16 core D14.4", T[2], base=[(model, bore)], StepOver=50,
         Direction="CW")
    b.op("Slot", "070 Slot dovetail rough", T[2], workplane=wp_right,
         base=[(model, [dove_floor])], ExtendPathStart=12.0, ExtendPathEnd=12.0)
    # finishing and slots
    b.op("PocketShape", "080 Pocket finish", T[3], base=[(model, [pocket_floor])], StepOver=40,
         FinishingPasses=1)
    b.op("Slot", "090 Slot T-slot neck", T[3], workplane=wp_front, base=[(model, tslot_neck)],
         ExtendPathStart=10.0, ExtendPathEnd=10.0, FinalDepth=-TSLOT_NECK_DEPTH, StepDown=3.0)
    b.op("Pocket", "100 3D pocket rough dish", T[4], base=[(model, dish)], StepOver=40,
         StepDown=2.0)
    b.op("Surface", "110 Surface semi-finish dish", T[5], base=[(model, dish)],
         CutPattern="Line", StepOver=12, DepthOffset=0.1, BoundBox="BaseBoundBox")
    b.op("PlanarSurface", "120 Planar surface finish dish", T[6], base=[(model, dish)],
         StepOver=8)
    # undercut tools: one pass at full depth, entering and leaving outside the part
    b.op("Slot", "130 Slot dovetail", T[16], workplane=wp_right, base=[(model, [dove_floor])],
         ExtendPathStart=15.0, ExtendPathEnd=15.0)
    b.op("Slot", "140 Slot T-slot undercut - slitting saw", T[17], workplane=wp_front,
         base=[(model, tslot_neck)], ExtendPathStart=12.0, ExtendPathEnd=12.0,
         StartDepth=-TSLOT_NECK_DEPTH, FinalDepth=-(TSLOT_NECK_DEPTH + SAW_T), StepDown=SAW_T)
    b.op("Custom", "145 Optional stop - inspect slots", None, coolant="None", Source="Text",
         Gcode=["(Optional stop - blow out chips, gauge the dovetail and T-slot)", "M9", "M1"])
    # holes
    b.op("Drilling", "150 Spot drill top holes", T[7], base=[(model, m8_top + dowels)],
         ExtraOffset="None", FinalDepth=H - 4.4)
    b.op("Drilling", "160 Spot drill front holes", T[7], workplane=wp_front,
         base=[(model, m8_front)], ExtraOffset="None", FinalDepth=-4.4)
    b.op("Drilling", "170 Drill M8 tap drill top", T[11], base=[(model, m8_top)],
         PeckEnabled=True, ChipBreakEnabled=True, ExtraOffset="Tool Tip")
    b.op("Drilling", "180 Drill M8 tap drill front", T[11], workplane=wp_front,
         base=[(model, m8_front)], PeckEnabled=True, ChipBreakEnabled=True,
         ExtraOffset="Tool Tip")
    b.op("Drilling", "190 Drill 6H7 pre-drill", T[12], base=[(model, dowels)], PeckEnabled=True,
         ExtraOffset="Tool Tip")
    b.op("Drilling", "200 Ream 6H7 dowel holes", T[13], base=[(model, dowels)],
         FeedRetractEnabled=True, ExtraOffset="None", FinalDepth=H - 11.0)
    b.op("Drilling", "210 Tap M8 top", T[14], base=[(model, m8_top)], Strategy="Tapping",
         ExtraOffset="None", FinalDepth=H - 10.0)
    b.op("Drilling", "220 Tap M8 front", T[14], workplane=wp_front, base=[(model, m8_front)],
         Strategy="Tapping", ExtraOffset="None", FinalDepth=-13.0)
    b.op("ThreadMilling", "230 Thread mill M16x1.5", T[15], base=[(model, bore)],
         ThreadType="MetricInternal6H", ThreadOrientation="RightHand", MajorDiameter=16.0,
         MinorDiameter=14.376, Pitch=1.5, Passes=2, Direction="Climb")
    # flutes and markings
    b.op("Flute", "240 Flute grip grooves", T[10], base=[(model, flute_bottoms)])
    # V-carve and Engrave take their default depths from the world, not the work plane
    b.op("Vcarve", "250 V-carve part number", T[9], workplane=wp_back, BaseShapes=[pn],
         coolant="Mist", StartDepth=0.0, FinalDepth=-1.5, StepDown=1.5)
    b.op("Engrave", "260 Engrave serial number", T[9], workplane=wp_front, BaseShapes=[sn],
         coolant="Mist", StartDepth=0.0, FinalDepth=-0.3, StepDown=0.3)
    # edge breaks
    b.op("Deburr", "270 Deburr pocket and bore", T[7], base=[(model, rims)], Width=0.5,
         ExtraDepth=0.3)
    # a corner rounding cutter as an endmill: pilot on the wall, tip R below the edge
    b.op("Deburr", "280 Round outside top edges R2", T[8], base=[(model, outer_top_edges)],
         Width=2.0, ExtraDepth=2.0)
    # fmt: on

    doc.recompute()
    ok = b.report()
    if os.path.exists(OUT):
        os.remove(OUT)  # replaced, not backed up next to it
    doc.saveAs(OUT)
    print("saved %s and %s" % (OUT, os.path.join(OUT_DIR, MACHINE_FILE)))
    return ok


try:
    build()
except Exception:
    traceback.print_exc()
if FreeCAD.GuiUp and os.environ.get("CAM_DEMO_EXIT"):
    sys.stdout.flush()
    os._exit(0)
