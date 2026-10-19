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

"""What the GL simulator is given for operations on work planes.

The simulator takes the tool direction from A, B, C only, draws arcs in the
world XY plane and runs drill cycles along world Z. These tests check that an
operation on a tilted plane reaches it in that form, and that the tool turns
from one plane to the next only clear of the stock.
"""

import math

import FreeCAD
from FreeCAD import Vector

import Path
import CAMTests.PathTestUtils as PathTestUtils
from Path.Main.SimulatorFeed import SimulatorFeed, rotaryForAxis, toolAxis, toolProfile

# a 100 x 80 x 40 block
BOUNDS = FreeCAD.BoundBox(0, 0, 0, 100, 80, 40)
TOOL_LENGTH = 30.0

TOP = FreeCAD.Placement()
# the front face, tool along -Y, plane X along world X
FRONT = FreeCAD.Placement(
    Vector(0, 0, 0), FreeCAD.Rotation(Vector(1, 0, 0), Vector(0, 0, 1), Vector(0, -1, 0), "ZXY")
)
# a 45 degree chamfer on the back top edge
CHAMFER = FreeCAD.Placement(
    Vector(50, 70, 30), FreeCAD.Rotation(Vector(0, 0, 1), Vector(0, 1, 1).normalize())
)


class _Op:
    def __init__(self, gcode, placement=TOP):
        self.Path = Path.Path([Path.Command(line) for line in gcode.strip().splitlines()])
        self.Placement = placement


def _point(cmd):
    return Vector(cmd.x, cmd.y, cmd.z)


def _axis(cmd):
    p = cmd.Parameters
    return toolAxis(p["A"], p["B"], p["C"])


class TestPathSimulatorFeed(PathTestUtils.PathTestBase):
    def assertVector(self, v1, v2, error=1e-6):
        self.assertTrue((v1 - v2).Length < error, f"{v1} != {v2}")

    def test00(self):
        """rotaryForAxis() points the tool where toolAxis() says it does."""
        for axis in [
            Vector(0, 0, 1),
            Vector(0, 0, -1),
            Vector(0, -1, 0),
            Vector(0, 1, 0),
            Vector(1, 0, 0),
            Vector(-1, 0, 0),
            Vector(0, 1, 1),
            Vector(1, -2, 3),
        ]:
            axis.normalize()
            self.assertVector(toolAxis(*rotaryForAxis(axis)), axis)

    def test01(self):
        """An operation in the Job's XY is unchanged but for every word filled in."""
        op = _Op(
            """
            G0 Z20
            G0 X10 Y10
            G1 Z-1
            G2 X20 Y10 I5 J0
            """
        )
        retract, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        self.assertEqual(retract, [])
        self.assertEqual([c.Name for c in cmds], ["G0", "G0", "G1", "G2"])
        self.assertVector(_point(cmds[0]), Vector(0, 0, 20))
        self.assertVector(_point(cmds[3]), Vector(20, 10, -1))
        self.assertRoughly(cmds[3].Parameters["I"], 5)
        for cmd in cmds:
            self.assertVector(_axis(cmd), Vector(0, 0, 1))

    def test02(self):
        """An operation on a tilted plane runs with the tool along the plane's Z."""
        op = _Op(
            """
            G0 X10 Y10 Z20
            G1 Z0
            G1 X30
            """,
            CHAMFER,
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        axis = CHAMFER.Rotation.multVec(Vector(0, 0, 1))
        for cmd in cmds:
            self.assertVector(_axis(cmd), axis)
        self.assertVector(_point(cmds[1]), CHAMFER.multVec(Vector(10, 10, 0)))
        self.assertVector(_point(cmds[2]), CHAMFER.multVec(Vector(30, 10, 0)))

    def test03(self):
        """A drill cycle on a tilted plane plunges and retracts along the plane's Z."""
        op = _Op(
            """
            G0 X20 Y20 Z15
            G98
            G81 X20 Y20 Z-10 R3 F100
            G81 X80 Y20 Z-10 R3 F100
            G80
            """,
            FRONT,
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        self.assertFalse([c for c in cmds if c.Name not in ("G0", "G1")])
        # every point is over one of the two holes, at a height on the plane's Z
        heights = set()
        for cmd in cmds:
            local = FRONT.inverse().multVec(_point(cmd))
            holes = (Vector(20, 20, 0), Vector(80, 20, 0))
            self.assertTrue(
                any((Vector(local.x, local.y, 0) - h).Length < 1e-6 for h in holes),
                f"{local} is off the holes",
            )
            heights.add(round(local.z, 6))
        self.assertEqual(heights, {15, 3, -10})
        # G98 goes back to the height the cycle started from before moving on
        travel = [
            (FRONT.inverse().multVec(_point(a)), FRONT.inverse().multVec(_point(b)))
            for a, b in zip(cmds, cmds[1:])
        ]
        for a, b in travel:
            if abs(a.x - b.x) > 1e-6:
                self.assertRoughly(a.z, 15)
                self.assertRoughly(b.z, 15)

    def test04(self):
        """G99 retracts to R between holes."""
        op = _Op(
            """
            G0 X20 Y20 Z15
            G99
            G81 X20 Y20 Z-10 R3 F100
            G81 X80 Y20 Z-10 R3 F100
            """,
            FRONT,
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        local = FRONT.inverse().multVec(_point(cmds[-1]))
        self.assertVector(local, Vector(80, 20, 3))

    def test05(self):
        """An arc on a tilted plane is followed by lines in that plane."""
        op = _Op(
            """
            G0 X10 Y20 Z-2
            G2 X30 Y20 I10 J0
            G3 X30 Y20 I-10 J0
            """,
            FRONT,
        )
        feed = SimulatorFeed(BOUNDS, tolerance=0.01)
        _, cmds = feed.add(op, TOOL_LENGTH)
        self.assertFalse([c for c in cmds if c.Name in Path.Geom.CmdMoveArc])
        self.assertGreater(len(cmds), 20)
        center = FRONT.multVec(Vector(20, 20, -2))
        normal = FRONT.Rotation.multVec(Vector(0, 0, 1))
        for cmd in cmds:
            offset = _point(cmd) - center
            self.assertRoughly(offset.dot(normal), 0, 1e-6)
            self.assertRoughly(offset.Length, 10, 1e-6)
        self.assertVector(_point(cmds[-1]), FRONT.multVec(Vector(30, 20, -2)))
        # clockwise from the left the G2 goes over the top of the circle in the plane
        firstHalf = cmds[1 : len(cmds) // 3]
        self.assertTrue(all(FRONT.inverse().multVec(_point(c)).y > 20 - 1e-6 for c in firstHalf))

    def test06(self):
        """An operation turned over keeps its arcs, with the direction swapped."""
        under = FreeCAD.Placement(Vector(0, 0, 0), FreeCAD.Rotation(Vector(1, 0, 0), 180))
        op = _Op(
            """
            G0 X10 Y20 Z0
            G2 X30 Y20 I10 J0
            """,
            under,
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        self.assertEqual(cmds[1].Name, "G3")
        self.assertVector(_point(cmds[1]), Vector(30, -20, 0))
        self.assertRoughly(cmds[1].Parameters["I"], 10)
        self.assertRoughly(cmds[1].Parameters["J"], 0)
        self.assertVector(_axis(cmds[1]), Vector(0, 0, -1))

    def test07(self):
        """Between work planes the tool backs out, turns clear of the stock, and comes in."""
        top = _Op(
            """
            G0 X50 Y40 Z45
            G1 Z38
            G0 Z45
            """
        )
        front = _Op(
            """
            G0 X50 Y20 Z5
            G1 Z-1
            """,
            FRONT,
        )
        feed = SimulatorFeed(BOUNDS)
        feed.add(top, TOOL_LENGTH)
        retract, cmds = feed.add(front, TOOL_LENGTH)

        # out along +Z from where the top operation ended
        self.assertEqual(len(retract), 1)
        self.assertVector(_axis(retract[0]), Vector(0, 0, 1))
        out = _point(retract[0])
        self.assertRoughly(out.x, 50)
        self.assertRoughly(out.y, 40)
        self.assertGreater(out.z, 45)

        # the front operation starts after the swing, which ends on its tool axis
        first = FRONT.multVec(Vector(50, 20, 5))
        start = next(i for i, c in enumerate(cmds) if (_point(c) - first).Length < 1e-6)
        swing = retract + cmds[:start]
        self.assertGreater(len(swing), 3)
        approach = _point(cmds[start - 1]) - first
        approach.normalize()
        self.assertVector(approach, Vector(0, -1, 0))
        for cmd in cmds[start:]:
            self.assertVector(_axis(cmd), Vector(0, -1, 0))

        # while the tool turns no part of it, its whole length included, comes near the stock
        center, reach = BOUNDS.Center, BOUNDS.DiagonalLength / 2 + TOOL_LENGTH
        points = [_point(c) for c in swing]
        turning = [
            (a, b)
            for a, b, ca, cb in zip(points, points[1:], swing, swing[1:])
            if _axis(ca).getAngle(_axis(cb)) > 1e-9
        ]
        self.assertTrue(turning)
        for a, b in zip(points, points[1:]):
            for k in range(11):
                p = a + (b - a) * (k / 10)
                self.assertGreater((p - center).Length, reach)

    def test08(self):
        """Operations on the same plane follow on from each other directly."""
        feed = SimulatorFeed(BOUNDS)
        feed.add(_Op("G0 X0 Y0 Z5\nG1 X10", FRONT), TOOL_LENGTH)
        retract, cmds = feed.add(_Op("G0 X20 Y0 Z5\nG1 X30", FRONT), TOOL_LENGTH)
        self.assertEqual(retract, [])
        self.assertEqual(len(cmds), 2)

    def test09(self):
        """A path with rotary words of its own keeps them, every word filled in."""
        op = _Op(
            """
            G0 X0 Y0 Z50 A0 B0
            G1 X10 B30
            G1 Y5
            """
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        self.assertEqual(len(cmds), 3)
        self.assertRoughly(cmds[1].Parameters["B"], 30)
        self.assertRoughly(cmds[2].Parameters["B"], 30)
        self.assertVector(_point(cmds[2]), Vector(10, 5, 50))

    def test10(self):
        """An operation with no path, or no moves, gives the simulator nothing."""
        feed = SimulatorFeed(BOUNDS)
        empty = _Op("")
        empty.Path = None
        self.assertEqual(feed.add(empty), ([], []))
        self.assertEqual(feed.add(_Op("M3 S1000")), ([], []))

    def test11(self):
        """Feed rates go with the moves that have them, arcs divided into lines and the
        plunges of drill cycles included."""
        op = _Op(
            """
            G0 X10 Y20 Z5
            G1 Z-2 F7
            G2 X30 Y20 I10 J0 F8
            G81 X40 Y20 Z-5 R2 F9
            G80
            """,
            FRONT,
        )
        _, cmds = SimulatorFeed(BOUNDS).add(op, TOOL_LENGTH)
        self.assertNotIn("F", cmds[0].Parameters)
        self.assertRoughly(cmds[1].Parameters["F"], 7)
        chords = [c for c in cmds[2:] if c.Parameters.get("F") == 8]
        self.assertGreater(len(chords), 10)
        plunges = [c for c in cmds if c.Parameters.get("F") == 9]
        self.assertEqual(len(plunges), 1)
        self.assertEqual(plunges[0].Name, "G1")
        self.assertFalse([c for c in cmds if c.Name == "G0" and "F" in c.Parameters])
        # in the Job's XY the arc keeps its feed rate
        _, cmds = SimulatorFeed(BOUNDS).add(_Op("G0 X0 Y0 Z0\nG3 X10 Y0 I5 J0 F4"), TOOL_LENGTH)
        self.assertRoughly(cmds[1].Parameters["F"], 4)

    def test12(self):
        """The profile of a tool solid has its arcs, with their centers in the profile."""
        import Part

        bullnose = Part.makeCylinder(5, 30)
        bottom = [e for e in bullnose.Edges if e.isClosed() and e.BoundBox.ZMax < 1e-9]
        bullnose = bullnose.makeFillet(2, bottom)
        profile, arcs = toolProfile(bullnose, 0.5)
        points = list(zip(profile[0::2], profile[1::2]))
        self.assertRoughly(points[0][0], 5)
        self.assertRoughly(points[0][1], 30)
        self.assertEqual(len(arcs), 1)
        first, last, r, z = arcs[0]
        self.assertRoughly(r, 3)
        self.assertRoughly(z, 2)
        self.assertGreaterEqual(last - first, 8)
        for pr, pz in points[first : last + 1]:
            self.assertRoughly(math.hypot(pr - 3, pz - 2), 2, 1e-6)
        self.assertRoughly(points[first][0], 5)
        self.assertRoughly(points[last][1], 0)

    def test13(self):
        """Arcs in the XZ and YZ planes are followed by lines in their plane."""
        op = _Op(
            """
            G0 X10 Y30 Z40
            G18
            G2 X56 Z40 I23 K64.125
            G19
            G3 Y40 Z40 J5 K0
            G17
            G1 X60
            """
        )
        _, cmds = SimulatorFeed(BOUNDS, tolerance=0.01).add(op, TOOL_LENGTH)
        self.assertFalse([c for c in cmds if c.Name in Path.Geom.CmdMoveArc])
        center = Vector(33, 30, 104.125)
        scoop = [c for c in cmds if abs(c.y - 30) < 1e-9 and c.x > 10]
        self.assertGreater(len(scoop), 10)
        for cmd in scoop:
            self.assertRoughly((_point(cmd) - center).Length, 68.125, 1e-6)
            # G2 in the XZ plane from X10 to X56 goes under its center, down into the part
            self.assertLessEqual(cmd.z, 40 + 1e-9)
        # G3 in the YZ plane from Y30 to Y40 around Y35: counterclockwise from +Y towards +Z
        # seen from +X, down through Z35
        loop = [c for c in cmds if abs(c.x - 56) < 1e-9 and c.y > 30 + 1e-6]
        self.assertGreater(len(loop), 10)
        self.assertRoughly(min(c.z for c in loop), 35, 0.011)
        self.assertVector(_point(cmds[-1]), Vector(60, 40, 40))
