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

"""The moves the GL simulator is given for a sequence of operations.

The simulator reads G-code in world coordinates: the tool tip position in X, Y,
Z and the tool direction in A, B, C (see MotionToolAxis in MillMotion.h). It
draws G2/G3 in the world XY plane and runs drill cycles along world Z.

An operation on a work plane stores its path in the plane's frame with none of
that. So each operation is given to the simulator as:

  * every move with its full X, Y, Z in the world and the tool direction of
    the work plane in A, B, C, and its feed rate (F) where it has one;
  * drill cycles expanded into the plain moves they make, in the plane's frame;
  * arcs on a tilted plane, and arcs in the XZ or YZ plane (G18, G19), split
    into short lines, since they don't lie in the world XY plane.

Between operations whose tool direction differs, the simulator turns the tool
over the length of a move, as a five axis machine does. Going straight from
one operation to the next would swing the tool through the part. The real
program retracts and cancels the plane before the table turns; the feed does
the same: the tool backs out along the old direction until it is clear of the
stock, swings around it at that distance, and comes in along the new one.

Before the first move the simulator doesn't know where the tool is: the
first moves take the axes they don't name from the first move that does, so
the tool starts where the program takes it, not at the origin.

An operation whose path carries rotary words of its own is simultaneous
machining. Its path is passed on as the operation's Placement puts it, with
only the drill cycles expanded and every word filled in.
"""

import math

import FreeCAD
import Path
from FreeCAD import Vector

import Constants
import Path.Base.Util as PathUtil

# the most a swing around the stock turns in one move, so its chords stay clear
_SWING_STEP = math.radians(15)
# how far from a vertical plane a work plane is still treated as one, so that
# its arcs can stay arcs
_AXIS_TOLERANCE = 1e-9
# the angle between two tool directions that counts as a change
_TURN_TOLERANCE = 1e-6

_DEFAULT_TOOL_LENGTH = 50.0
_RETURN_MODES = {"G98", "G99"}
_PLANES = {"G17": "G17", "G18": "G18", "G19": "G19"}
# the axes of the arcs of a plane: (u, v), counterclockwise from u towards v
_PLANE_AXES = {"G17": ("x", "y"), "G18": ("z", "x"), "G19": ("y", "z")}
_CENTER_WORDS = {"x": "I", "y": "J", "z": "K"}
_CYCLES = set(Constants.GCODE_DRILL_COMMANDS + Constants.GCODE_MOVE_TAP)
_MOVES = set(Constants.GCODE_MOVE_LINE + Constants.GCODE_MOVE_ARC) | _CYCLES


def toolAxis(a, b, c):
    """The tool direction for rotary positions in degrees, as MotionToolAxis has it:
    the vertical axis turned by A about X, then B about Y, then C about Z."""
    rotation = (
        FreeCAD.Rotation(Vector(0, 0, 1), c)
        * FreeCAD.Rotation(Vector(0, 1, 0), b)
        * FreeCAD.Rotation(Vector(1, 0, 0), a)
    )
    return rotation.multVec(Vector(0, 0, 1))


def rotaryForAxis(axis, near=(0.0, 0.0, 0.0)):
    """A, B, C in degrees that point the tool along axis, with C at 0. Each angle is
    taken as close to the one in near as the turn allows, so a move between
    them does not go the long way round."""
    u = Vector(axis)
    u.normalize()
    a = -math.degrees(math.asin(max(-1.0, min(1.0, u.y))))
    b = math.degrees(math.atan2(u.x, u.z)) if abs(abs(u.y) - 1) > _AXIS_TOLERANCE else near[1]
    return (_nearest(a, near[0]), _nearest(b, near[1]), _nearest(0.0, near[2]))


def _nearest(angle, near):
    return angle + 360.0 * round((near - angle) / 360.0)


def _move(name, point, rotary, extra=None):
    params = {"X": point.x, "Y": point.y, "Z": point.z}
    if extra:
        params.update(extra)
    params.update({"A": rotary[0], "B": rotary[1], "C": rotary[2]})
    return Path.Command(name, params)


class SimulatorFeed:
    """Turns operations, one after the other, into what the simulator is given.

    bounds is the bounding box of the stock: the tool turns from one work plane
    to the next only where it cannot reach it."""

    def __init__(self, bounds, tolerance=0.01):
        self.center = Vector(bounds.Center)
        self.radius = bounds.DiagonalLength / 2
        self.tolerance = tolerance
        self.position = Vector(0, 0, 0)
        self.rotary = (0.0, 0.0, 0.0)
        self.toolLength = None
        self.started = False

    def add(self, op, toolLength=None):
        """add(op, toolLength=None) ... returns (retract, commands) for op.

        retract takes the tool out of the previous operation, with the previous
        tool; commands bring it to op and run op. toolLength is the length of
        op's tool, None to keep the previous one."""
        moves = self._moves(op)
        if not moves:
            return [], []

        length = max(self.toolLength or 0, toolLength or 0) or _DEFAULT_TOOL_LENGTH
        if toolLength:
            self.toolLength = toolLength

        retract, approach = [], []
        first = moves[0].Parameters
        rotary = (first["A"], first["B"], first["C"])
        if self.started and self._turns(self.rotary, rotary):
            retract, approach = self._reorient(
                Vector(first["X"], first["Y"], first["Z"]), rotary, length
            )
        self.started = True

        last = moves[-1].Parameters
        self.position = Vector(last["X"], last["Y"], last["Z"])
        self.rotary = (last["A"], last["B"], last["C"])
        return retract, approach + moves

    @staticmethod
    def _turns(rotary0, rotary1):
        axis0, axis1 = toolAxis(*rotary0), toolAxis(*rotary1)
        return axis0.getAngle(axis1) > _TURN_TOLERANCE

    def _moves(self, op):
        """op's moves in the world, every word filled in."""
        path = getattr(op, "Path", None)
        if path is None:
            return []
        commands = path.Commands
        if any(k in c.Parameters for c in commands for k in "ABC"):
            return self._walk(PathUtil.getPathWithPlacement(op).Commands, FreeCAD.Placement(), None)

        placement = getattr(op, "Placement", None) or FreeCAD.Placement()
        axis = placement.Rotation.multVec(Vector(0, 0, 1))
        return self._walk(commands, placement, rotaryForAxis(axis, self.rotary))

    def _walk(self, commands, placement, rotary):
        """The moves of commands in the world. commands are in the frame placement
        puts in the world; rotary is the tool direction for all of them, or None
        for commands that carry their own."""
        out = []
        frame = placement.inverse()
        local = frame.multVec(self.position)
        current = self.rotary if rotary is None else rotary
        if not self.started:
            # where the tool is before its first move: each axis where a move first names it
            first = {}
            for cmd in commands:
                if cmd.Name.upper() in _MOVES:
                    for k in "XYZABC":
                        if k in cmd.Parameters:
                            first.setdefault(k, cmd.Parameters[k])
            local = Vector(first.get("X", local.x), first.get("Y", local.y), first.get("Z", local.z))
            if rotary is None:
                current = tuple(first.get(k, v) for k, v in zip("ABC", current))
        axis = placement.Rotation.multVec(Vector(0, 0, 1))
        flat = abs(abs(axis.z) - 1) < _AXIS_TOLERANCE
        turnedOver = axis.z < 0
        returnMode = "G98"
        plane = "G17"

        def emit(name, point, extra=None):
            out.append(_move(name, placement.multVec(point), current, extra))

        for cmd in commands:
            name = cmd.Name.upper()
            params = cmd.Parameters
            if name in _RETURN_MODES:
                returnMode = name
                continue
            if name in _PLANES:
                plane = name
                continue
            # a cycle that cannedCycleTerminator() has taken the G98 or G99 into
            returnMode = cmd.Annotations.get("RetractMode", returnMode)
            if name not in _MOVES:
                continue

            if rotary is None:
                current = tuple(params.get(k, v) for k, v in zip("ABC", current))
            target = Vector(
                params.get("X", local.x), params.get("Y", local.y), params.get("Z", local.z)
            )
            feed = {"F": params["F"]} if "F" in params else {}

            if name in _CYCLES:
                # the cycle's own moves, along the frame's Z: to the hole at the
                # height it starts from, to R, down to Z and back to the plane
                # the return mode names
                r = params.get("R", local.z)
                back = r if returnMode == "G99" else max(local.z, r)
                above = max(local.z, r)
                if local.z < above:
                    emit("G0", Vector(local.x, local.y, above))
                emit("G0", Vector(target.x, target.y, above))
                emit("G0", Vector(target.x, target.y, r))
                emit("G1", target, feed)
                emit("G0", Vector(target.x, target.y, back))
                local = Vector(target.x, target.y, back)
                continue

            if name in Path.Geom.CmdMoveArc:
                if flat and plane == "G17":
                    offset = placement.Rotation.multVec(
                        Vector(params.get("I", 0), params.get("J", 0), 0)
                    )
                    if turnedOver:
                        name = {"G2": "G3", "G02": "G03", "G3": "G2", "G03": "G02"}[name]
                    emit(name, target, {"I": offset.x, "J": offset.y, **feed})
                else:
                    for point in self._arcPoints(name, params, local, target, plane):
                        emit("G1", point, feed)
                local = target
                continue

            emit("G0" if name in Path.Geom.CmdMoveRapid else "G1", target, feed)
            local = target

        return out

    def _arcPoints(self, name, params, start, end, plane="G17"):
        """The points, after start, of lines that follow an arc in the plane (G17, G18
        or G19) within tolerance, helical along the normal of the plane."""
        u, v = _PLANE_AXES[plane]
        cu = getattr(start, u) + params.get(_CENTER_WORDS[u], 0)
        cv = getattr(start, v) + params.get(_CENTER_WORDS[v], 0)
        radius = math.hypot(getattr(start, u) - cu, getattr(start, v) - cv)
        a0 = math.atan2(getattr(start, v) - cv, getattr(start, u) - cu)
        a1 = math.atan2(getattr(end, v) - cv, getattr(end, u) - cu)
        sweep = a1 - a0
        clockwise = name in Path.Geom.CmdMoveCW
        if clockwise:
            sweep = -((-sweep) % (2 * math.pi))
        else:
            sweep = sweep % (2 * math.pi)
        if abs(sweep) < 1e-9:
            sweep = -2 * math.pi if clockwise else 2 * math.pi

        if radius > self.tolerance:
            step = 2 * math.acos(1 - self.tolerance / radius)
        else:
            step = math.pi / 2
        n = max(1, math.ceil(abs(sweep) / step))
        points = []
        for k in range(1, n + 1):
            t = k / n
            angle = a0 + sweep * t
            point = start + (end - start) * t
            setattr(point, u, cu + radius * math.cos(angle))
            setattr(point, v, cv + radius * math.sin(angle))
            points.append(point)
        points[-1] = Vector(end)
        return points

    def _clearance(self, length):
        """The distance from the centre of the stock at which no part of a tool of
        length, at any angle, reaches it, plus room for the swing's chords."""
        margin = max(1.0, 0.05 * self.radius)
        return (self.radius + length + margin) / math.cos(_SWING_STEP / 2)

    def _outward(self, point, axis, distance):
        """point moved along axis until it is distance from the centre of the stock."""
        offset = point - self.center
        b = offset.dot(axis)
        c = offset.dot(offset) - distance * distance
        if c >= 0:
            return Vector(point)
        return point + axis * (-b + math.sqrt(b * b - c))

    def _reorient(self, target, rotary, length):
        """The moves from where the tool is to target, turning it to rotary.

        Returns (retract, approach): retract backs out along the old tool
        direction; approach swings around the stock at the clearance distance,
        turning the tool, and stops above target on the new tool direction."""
        distance = self._clearance(length)
        axis0, axis1 = toolAxis(*self.rotary), toolAxis(*rotary)
        p1 = self._outward(self.position, axis0, distance)
        q1 = self._outward(target, axis1, distance)
        retract = [] if p1.isEqual(self.position, 1e-9) else [_move("G0", p1, self.rotary)]

        # out to the larger of the two distances, around at it, and in
        d0, d1 = p1 - self.center, q1 - self.center
        r = max(d0.Length, d1.Length)
        approach = []
        if r - d0.Length > 1e-9:
            approach.append(_move("G0", self.center + d0 * (r / d0.Length), self.rotary))
        d0.normalize()
        d1.normalize()
        angle = d0.getAngle(d1)
        n = max(1, math.ceil(angle / _SWING_STEP))
        for k in range(1, n + 1):
            t = k / n
            direction = _slerp(d0, d1, angle, t)
            turn = tuple(v0 + (v - v0) * t for v0, v in zip(self.rotary, rotary))
            approach.append(_move("G0", self.center + direction * r, turn))
        if not (self.center + d1 * r).isEqual(q1, 1e-9):
            approach.append(_move("G0", q1, rotary))
        return retract, approach


def _radiusAt(edge, p):
    """The distance of a point of an edge from the tool axis."""
    v = edge.valueAt(p)
    return math.hypot(v.x, v.y)


def _isSame(x, y):
    return abs(x - y) < 0.0001


def _topMostEdge(edges):
    """The edge with the highest end, and its parameters from that end."""
    best, top = (None, 0.0, 0.0), -math.inf
    for edge in edges:
        p1, p2 = edge.FirstParameter, edge.LastParameter
        for a, b in ((p1, p2), (p2, p1)):
            z = edge.valueAt(a).z
            if z > top:
                best, top = (edge, a, b), z
    return best


def _closestEdge(edges, r, z):
    """The edge starting at (r, z) in the profile, and its parameters from there. A flat circle
    may have no edge: then an edge at the same height, joined to it by a line."""
    for edge in edges:
        p1, p2 = edge.FirstParameter, edge.LastParameter
        r1, z1 = _radiusAt(edge, p1), edge.valueAt(p1).z
        if _isSame(r, r1) and _isSame(z, z1):
            return edge, p1, p2
        r2, z2 = _radiusAt(edge, p2), edge.valueAt(p2).z
        if _isSame(r, r2) and _isSame(z, z2):
            return edge, p2, p1
        if _isSame(z, z1):
            return edge, p1, p2
        if _isSame(z, z2):
            return edge, p2, p1
    return None, 0.0, 0.0


def toolProfile(shape, resolution):
    """toolProfile(shape, resolution) ... (profile, arcs) of the solid of revolution of a tool
    around its Z axis, as the simulator takes them.

    profile is the side of the solid from its top down to its tip as radius/height pairs
    [r0, z0, r1, z1, ...], arcs divided into lines of at most resolution and at most 1/32 of a
    half circle. arcs are the circles of the side, (first, last, r, z): the points first to last
    of the profile are on the circle around (r, z), the simulator follows them exactly."""
    import Part

    shape = shape.copy()
    shape.Placement = FreeCAD.Placement()
    # the edges of the side, along the seam of the revolution
    edges = [e for e in shape.Edges if not e.isClosed()]
    edge, p1, p2 = _topMostEdge(edges)
    if edge is None:
        return [], []
    profile = [_radiusAt(edge, p1), edge.valueAt(p1).z]
    arcs = []
    r, z = profile
    while edge is not None:
        edges.remove(edge)
        if isinstance(edge.Curve, Part.Circle):
            n = max(int(edge.Length / resolution) + 1, math.ceil(abs(p2 - p1) / (math.pi / 32)))
            first = len(profile) // 2 - 1
            for k in range(1, n + 1):
                p = p1 + (p2 - p1) * k / n
                r, z = _radiusAt(edge, p), edge.valueAt(p).z
                profile += [r, z]
            # the center in the plane of the seam: its distance from the axis towards the edge
            mid = edge.valueAt((p1 + p2) / 2)
            length = math.hypot(mid.x, mid.y)
            center = edge.Curve.Center
            if length > 1e-12:
                cr = (center.x * mid.x + center.y * mid.y) / length
            else:
                cr = math.hypot(center.x, center.y)
            arcs.append((first, len(profile) // 2 - 1, cr, center.z))
        else:
            r, z = _radiusAt(edge, p2), edge.valueAt(p2).z
            profile += [r, z]
        edge, p1, p2 = _closestEdge(edges, r, z)
        if edge is None:
            break
        start = _radiusAt(edge, p1)
        if not _isSame(start, r):
            r, z = start, edge.valueAt(p1).z
            profile += [r, z]
    return profile, arcs


def _slerp(d0, d1, angle, t):
    """The unit vector a fraction t of the way from d0 to d1 on a great circle."""
    if angle < 1e-9:
        return Vector(d0)
    if math.pi - angle < 1e-6:
        # opposite: any great circle does, take one through a perpendicular
        normal = d0.cross(Vector(0, 0, 1))
        if normal.Length < 1e-6:
            normal = d0.cross(Vector(1, 0, 0))
        normal.normalize()
        side = normal.cross(d0)
        return d0 * math.cos(angle * t) + side * math.sin(angle * t)
    s = math.sin(angle)
    return d0 * (math.sin((1 - t) * angle) / s) + d1 * (math.sin(t * angle) / s)
