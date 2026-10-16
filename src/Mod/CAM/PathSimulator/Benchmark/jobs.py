# SPDX-License-Identifier: LGPL-2.1-or-later

"""Synthetic CAM jobs for benchmarking the CAM simulator.

Every job is a dict with:
    name      job name
    stock     (x, y, z, length, width, height) of a box stock
    tools     list of (tool number, diameter, profile) where profile is a flat list of
              radius/height pairs from the top of the tool down to the tip, in the format created
              by CAMSimulation.GetToolProfile
    gcode     list of G-code lines
    sections  list of (operation name, index of its first G-code line)

Operations are laid out so that they cut material, like on a real part: every operation works on
its own area of the stock, or on features created by an earlier operation. Run this file with
'analyze' to check how much material each operation removes.

The jobs only depend on the Python standard library, so they can also be inspected or exported
outside of FreeCAD. The analysis needs numpy.

Document jobs are the CAM jobs of FreeCAD documents (DOCUMENT_JOBS). They have one size, the
document's, and are read inside FreeCAD by sim_benchmark.py.

    python3 jobs.py                      list all jobs and their sizes
    python3 jobs.py analyze mixed small  material removed by every operation of a job
"""

import math
import os
import sys

# Target number of motion commands per job size
SIZES = {
    "small": 2_000,
    "medium": 20_000,
    "large": 100_000,
    "huge": 400_000,
}

STOCK = (0.0, 0.0, -20.0, 100.0, 100.0, 20.0)
FULL_AREA = (5.0, 5.0, 95.0, 95.0)
SHANK_TOP = 40.0


# Tool profiles. FreeCAD tool bits have their tip at z = 0.


def flat_endmill(diameter):
    r = diameter / 2
    return [r, SHANK_TOP, r, 0.0, 0.0, 0.0]


def ball_endmill(diameter, segments=8):
    r = diameter / 2
    profile = [r, SHANK_TOP, r, r]
    for i in range(1, segments + 1):
        a = i * (math.pi / 2) / segments
        profile += [r * math.cos(a), r - r * math.sin(a)]
    return profile


def bullnose(diameter, corner_radius, segments=6):
    r = diameter / 2
    flat = r - corner_radius
    profile = [r, SHANK_TOP, r, corner_radius]
    for i in range(1, segments + 1):
        a = i * (math.pi / 2) / segments
        profile += [flat + corner_radius * math.cos(a), corner_radius - corner_radius * math.sin(a)]
    profile += [0.0, 0.0]
    return profile


def vbit(diameter, angle_deg, tip_diameter=0.0):
    r = diameter / 2
    tip_r = tip_diameter / 2
    height = (r - tip_r) / math.tan(math.radians(angle_deg) / 2)
    return [r, SHANK_TOP, r, height, tip_r, 0.0, 0.0, 0.0]


def drill(diameter, tip_angle_deg=118):
    r = diameter / 2
    height = r / math.tan(math.radians(tip_angle_deg) / 2)
    return [r, SHANK_TOP, r, height, 0.0, 0.0]


def dovetail(diameter, neck_diameter, height, angle_deg=60):
    """Undercut: the cutting diameter grows towards the tip."""
    r = diameter / 2
    neck = neck_diameter / 2
    cut_height = min((r - neck) * math.tan(math.radians(angle_deg)), height)
    return [neck, SHANK_TOP, neck, cut_height, r, 0.0, 0.0, 0.0]


def slitting_saw(diameter, thickness, shank_diameter):
    """Undercut: a thin disc at the bottom of a narrow shank."""
    r = diameter / 2
    s = shank_diameter / 2
    return [s, SHANK_TOP, s, thickness, r, thickness, r, 0.0, 0.0, 0.0]


def thread_mill(diameter, neck_diameter, pitch, teeth):
    """Undercut: a stack of V shaped teeth below a narrow neck."""
    r = diameter / 2
    neck = neck_diameter / 2
    root = r - pitch * 0.6
    z = teeth * pitch
    profile = [neck, SHANK_TOP, neck, z]
    for _ in range(teeth):
        profile += [root, z, r, z - pitch / 2, root, z - pitch]
        z -= pitch
    profile += [0.0, 0.0]
    return profile


def corner_radius(radius, pilot_diameter, pilot_length=2.0, segments=6):
    """Corner rounding cutter: a pilot below a concave quarter circle cutting edge."""
    pilot = pilot_diameter / 2
    r = pilot + radius
    top = pilot_length + radius
    profile = [pilot + 1.0, SHANK_TOP, pilot + 1.0, top + 1.0, r, top + 1.0, r, top]
    for i in range(1, segments + 1):
        a = (math.pi / 2) * (1 - i / segments)
        profile += [pilot + radius * (1 - math.cos(a)), pilot_length + radius * math.sin(a)]
    profile += [pilot, 0.0, 0.0, 0.0]
    return profile


def tool_axis(a, b, c):
    """Tool axis for rotary axes in degrees, like MotionToolAxis of the simulator: the Z axis
    rotated by A around X, then B around Y, then C around Z."""
    a, b, c = math.radians(a), math.radians(b), math.radians(c)
    x1, y1, z1 = 0.0, -math.sin(a), math.cos(a)
    x2, y2, z2 = x1 * math.cos(b) + z1 * math.sin(b), y1, -x1 * math.sin(b) + z1 * math.cos(b)
    return (x2 * math.cos(c) - y2 * math.sin(c), x2 * math.sin(c) + y2 * math.cos(c), z2)


# G-code helpers


def _fmt(v):
    return f"{v:.4f}".rstrip("0").rstrip(".")


class _Writer:
    def __init__(self):
        self.lines = []
        self.tools = []
        self.sections = []

    def section(self, name):
        self.sections.append((name, len(self.lines)))

    def tool(self, number, diameter, profile):
        if number not in [t[0] for t in self.tools]:
            self.tools.append((number, diameter, profile))
        self.lines.append(f"M6 T{number}")

    def rapid(self, x=None, y=None, z=None, a=None, b=None, c=None):
        self.lines.append("G0" + self._xyz(x, y, z) + self._abc(a, b, c))

    def line(self, x=None, y=None, z=None, a=None, b=None, c=None):
        self.lines.append("G1" + self._xyz(x, y, z) + self._abc(a, b, c))

    def arc(self, cw, x, y, i, j, z=None):
        cmd = "G2" if cw else "G3"
        self.lines.append(f"{cmd}{self._xyz(x, y, z)} I{_fmt(i)} J{_fmt(j)}")

    def drill(self, x, y, z, r):
        self.lines.append(f"G81 X{_fmt(x)} Y{_fmt(y)} Z{_fmt(z)} R{_fmt(r)}")

    def retract(self):
        self.rapid(z=5)

    def rectangle(self, x0, y0, x1, y1):
        for x, y in ((x1, y0), (x1, y1), (x0, y1), (x0, y0)):
            self.line(x, y)

    @staticmethod
    def _xyz(x, y, z):
        s = ""
        if x is not None:
            s += f" X{_fmt(x)}"
        if y is not None:
            s += f" Y{_fmt(y)}"
        if z is not None:
            s += f" Z{_fmt(z)}"
        return s

    @staticmethod
    def _abc(a, b, c):
        s = ""
        for letter, value in (("A", a), ("B", b), ("C", c)):
            if value is not None:
                s += f" {letter}{_fmt(value)}"
        return s

    def job(self, name):
        return {
            "name": name,
            "stock": STOCK,
            "tools": self.tools,
            "gcode": self.lines,
            "sections": self.sections,
        }


def _shrink(area, d):
    x0, y0, x1, y1 = area
    return (x0 + d, y0 + d, x1 - d, y1 - d)


# Operations. Every operation cuts within its area (x0, y0, x1, y1) of the stock.


def _pocket_level(w, area, z, stepover, finish=0.5):
    """One level of a pocket: zigzag clearing, leaving 'finish' on the walls for a final pass
    along them.

    The area is the tool center area of the final pass.
    """
    x0, y0, x1, y1 = _shrink(area, finish)
    w.retract()
    w.rapid(x0, y0)
    w.line(z=z)
    y = y0
    while y < y1:
        w.line(x1, y)
        y2 = min(y + stepover, y1)
        w.line(x1, y2)
        w.line(x0, y2)
        y = y2 + stepover
        if y <= y1:
            w.line(x0, y)
    fx0, fy0, fx1, fy1 = area
    w.line(fx0, fy0)
    w.rectangle(fx0, fy0, fx1, fy1)


def op_pocket(w, area, n_moves, depth=12.0):
    """Pocket with many step down levels.

    Uses about n_moves moves, within realistic limits: step down at least 0.5, stepover at least
    1.0. Larger budgets than those limits allow are not used.
    """
    w.section("pocket")
    w.tool(1, 6.0, flat_endmill(6.0))
    inner = _shrink(area, 3.0)
    max_levels = int(depth / 0.5)
    for stepover in (2.0, 1.5, 1.0):
        probe = _Writer()
        _pocket_level(probe, inner, 0, stepover)
        levels = n_moves // len(probe.lines)
        if levels <= max_levels:
            break
    levels = min(max(2, levels), max_levels)
    for level in range(1, levels + 1):
        _pocket_level(w, inner, -depth * level / levels, stepover)
    w.retract()


def op_adaptive(w, area, n_moves, depth=9.0):
    """Trochoidal slots side by side, at several depths: lots of small arcs.

    Uses about n_moves moves, within realistic limits: step down at least 1.0, advance per loop at
    least 0.5. Larger budgets than those limits allow are not used.
    """
    w.section("adaptive")
    w.tool(3, 4.0, flat_endmill(4.0))
    x0, y0, x1, y1 = area
    r = 2.0
    width = 2 * r + 4.0  # slot width
    pitch = width - 1.0
    ys = []
    y = y0 + width / 2
    while y <= y1 - width / 2:
        ys.append(y)
        y += pitch
    start, end = x0 + width / 2, x1 - width / 2
    max_loops = int((end - start) / 0.5)
    levels = 3
    loops = n_moves // (3 * len(ys) * levels)
    while loops > max_loops and levels < int(depth):
        levels += 1
        loops = n_moves // (3 * len(ys) * levels)
    loops = min(max(2, loops), max_loops)
    advance = (end - start) / loops
    for level in range(1, levels + 1):
        for y in ys:
            w.retract()
            w.rapid(start, y - r)
            w.line(z=-depth * level / levels)
            x = start
            for _ in range(loops):
                w.arc(False, x, y + r, 0, r)
                w.arc(False, x, y - r, 0, -r)
                x += advance
                w.line(x, y - r)
    w.retract()


def _surface_z(x, y):
    return -6.0 + 2.5 * math.sin(x / 9.0) * math.cos(y / 13.0) + 1.5 * math.sin((x + y) / 21.0)


def op_surface(w, area, n_moves):
    """Ball end raster semi finishing and finishing of a free form surface."""
    w.section("surface")
    w.tool(2, 6.0, ball_endmill(6.0))
    x0, y0, x1, y1 = _shrink(area, 3.0)
    n = max(10, int(math.sqrt(n_moves / 2)))
    for allowance in (1.0, 0.0):
        w.retract()
        w.rapid(x0, y0)
        for row in range(n):
            y = y0 + (y1 - y0) * row / (n - 1)
            xs = [x0 + (x1 - x0) * i / (n - 1) for i in range(n)]
            if row % 2:
                xs.reverse()
            for x in xs:
                w.line(x, y, _surface_z(x, y) + allowance)
    w.retract()


def op_tool_shapes(w, area, n_moves):
    """A pocket machined with every tool shape FreeCAD supports, including undercut tools.

    The pocket is roughed and floor finished, gets ball end channels, a dovetail undercut and a
    slitting saw groove in its walls, rounded top edges and a threaded hole in its floor. A
    V groove surrounds it, and holes are drilled next to it. Needs an area of at least 40 x 40.
    """
    x0, y0, x1, y1 = area
    reps = max(1, n_moves // 400)
    pocket = (x0 + 6, y0 + 6, x1 - 6, y1 - 6)  # pocket walls
    px0, py0, px1, py1 = pocket
    cx, cy = (px0 + px1) / 2, (py0 + py1) / 2
    floor = -10.0

    w.section("rough pocket")
    w.tool(10, 8.0, flat_endmill(8.0))
    for level in range(1, 4):
        _pocket_level(w, _shrink(pocket, 4.0), floor * level / 3, 3.0)
    w.retract()

    w.section("bullnose floor")
    w.tool(11, 6.0, bullnose(6.0, 1.0))
    for k in range(reps):
        _pocket_level(w, _shrink(pocket, 3.0), floor - 0.3 * (k + 1), 4.0)
    floor -= 0.3 * reps
    w.retract()

    w.section("ball channels")
    w.tool(12, 6.0, ball_endmill(6.0))
    rows = 4 * reps
    for row in range(rows):
        y = py0 + 4 + (py1 - py0 - 8) * row / max(1, rows - 1)
        w.retract()
        w.rapid(px0 + 4, y)
        w.line(z=floor - 1.0)
        w.line(px1 - 4, y, floor - 1.0 - 0.5 * math.sin(row))
    w.retract()

    w.section("dovetail undercut")
    w.tool(14, 14.0, dovetail(14.0, 6.0, 6.0))
    for k in range(min(reps, 4)):
        offset = 5.0 - 0.4 * k
        w.retract()
        w.rapid(cx, cy)
        w.line(z=floor + 0.2)
        w.line(px0 + offset, py0 + offset)
        w.rectangle(px0 + offset, py0 + offset, px1 - offset, py1 - offset)
    w.retract()

    w.section("slitting saw groove")
    saw_radius, shank_radius = 15.0, 4.0
    w.tool(15, 2 * saw_radius, slitting_saw(2 * saw_radius, 2.0, 2 * shank_radius))
    w.retract()
    w.rapid(cx, cy)
    w.line(z=-5.0)
    half = min(px1 - px0, py1 - py0) / 2  # distance from the pocket center to the walls
    for k in range(min(reps, 3)):
        # the disc reaches 1 + k into the walls, the shank stays clear of them
        a = min(half - saw_radius + 1.0 + k, half - shank_radius - 1.0)
        w.line(cx - a, cy - a)
        w.rectangle(cx - a, cy - a, cx + a, cy + a)
    w.retract()

    w.section("corner radius")
    w.tool(17, 12.0, corner_radius(3.0, 6.0))
    w.retract()
    w.rapid(px0 + 3, py0 + 3)
    w.line(z=-5.0)
    w.rectangle(px0 + 3, py0 + 3, px1 - 3, py1 - 3)
    w.retract()

    w.section("bore")
    w.tool(10, 8.0, flat_endmill(8.0))
    w.retract()
    w.rapid(cx + 2, cy)
    w.line(z=floor)
    z = floor
    while z > -18:
        z = max(z - 1.0, -18.0)
        w.arc(False, cx - 2, cy, -2, 0, (z + (z + 1.0)) / 2)
        w.arc(False, cx + 2, cy, 2, 0, z)
    w.arc(False, cx - 2, cy, -2, 0)
    w.arc(False, cx + 2, cy, 2, 0)
    w.retract()

    w.section("thread mill")
    # one helix turn of one pitch cuts the thread, every tooth follows the previous one
    pitch = 1.5
    w.tool(16, 10.0, thread_mill(10.0, 6.0, pitch, 4))
    w.retract()
    w.rapid(cx, cy)
    z = -17.0
    w.line(z=z)
    w.line(cx + 2, cy)
    for x, y, i, j in ((cx, cy + 2, -2, 0), (cx - 2, cy, 0, 2), (cx, cy - 2, 2, 0)):
        z += pitch / 4
        w.arc(False, x, y, i, j, z)
    z += pitch / 4
    w.arc(False, cx + 2, cy, 0, 2, z)
    w.line(cx, cy)
    w.retract()

    w.section("v groove")
    w.tool(13, 10.0, vbit(10.0, 90))
    w.retract()
    w.rapid(x0 + 2, y0 + 2)
    w.line(z=-1.5)
    w.rectangle(x0 + 2, y0 + 2, x1 - 2, y1 - 2)
    w.retract()

    w.section("drill")
    w.tool(18, 5.0, drill(5.0))
    w.retract()
    n = 4
    for i in range(n):
        w.drill(x0 + 2, py0 + 4 + (py1 - py0 - 8) * i / (n - 1), -12, 2)
    w.retract()


# 5 axis operations


class _Dome:
    """A dome of radius 'radius' and height 'height' on a floor at 'base', centered in an area."""

    def __init__(self, area, radius=10.0, height=6.0, base=-9.0):
        self.cx = (area[0] + area[2]) / 2
        self.cy = (area[1] + area[3]) / 2
        self.radius = radius
        self.height = height
        self.base = base

    def z(self, x, y):
        r = min(1.0, math.hypot(x - self.cx, y - self.cy) / self.radius)
        return self.base + self.height * math.cos(r * math.pi / 2)

    def normal(self, x, y):
        e = 1e-3
        dx = (self.z(x + e, y) - self.z(x - e, y)) / (2 * e)
        dy = (self.z(x, y + e) - self.z(x, y - e)) / (2 * e)
        n = math.sqrt(dx * dx + dy * dy + 1)
        return (-dx / n, -dy / n, 1 / n), (dx, dy)

    def center_heights(self, ball, step=0.02, extent=None):
        """Height of the center of a ball of radius 'ball' touching the dome from above, by its
        distance from the dome axis, in steps of 'step': the drop cutter of the ball. The dome is
        rotationally symmetric, so the ball touches it in the plane through the axis and the
        center."""
        extent = extent or self.radius + 2 * ball
        table = []
        samples = int(2 * ball / step)
        for i in range(int(extent / step) + 2):
            rc = i * step
            best = -1e9
            for k in range(samples + 1):
                d = -ball + 2 * ball * k / samples  # along the radial line through the center
                r = abs(rc + d)
                z = self.base + self.height * math.cos(min(1.0, r / self.radius) * math.pi / 2)
                best = max(best, z + math.sqrt(max(0.0, ball * ball - d * d)))
            table.append(best)
        return table, step

    def island_radius(self, z):
        """Radius of the dome at height z."""
        if z <= self.base:
            return self.radius
        if z >= self.base + self.height:
            return 0.0
        return self.radius * 2 / math.pi * math.acos((z - self.base) / self.height)


def rotary_for_axis(axis):
    """A and B (C = 0) in degrees for a tool axis, the inverse of tool_axis."""
    ax, ay, az = axis
    return math.degrees(-math.asin(max(-1.0, min(1.0, ay)))), math.degrees(math.atan2(ax, az))


def op_dome(w, area, n_moves, lead=10.0, max_tilt=20.0):
    """A dome: roughed with waterline rings around it, then finished with a ball end mill whose
    axis follows the surface normal, leaning forward by the lead angle in the feed direction and
    limited to max_tilt from vertical: simultaneous 5 axis motion along every row."""
    dome = _Dome(area)
    half = min(area[2] - area[0], area[3] - area[1]) / 2

    w.section("dome roughing")
    rough_radius = 3.0
    w.tool(25, 2 * rough_radius, flat_endmill(2 * rough_radius))
    floor = dome.base
    outer = half - 0.5 - rough_radius  # ring centers, the pocket wall half a mm inside the area
    w.rapid(z=10)
    levels = 6
    for level in range(1, levels + 1):
        z = floor * level / levels
        inner = dome.island_radius(z) + 1.0 + rough_radius  # leave 1 for finishing
        # rings from the wall inwards, the last one at the finishing allowance
        rings = []
        r = outer
        while r > inner:
            rings.append(r)
            r -= 2.5
        rings.append(inner)
        w.rapid(dome.cx + outer, dome.cy)
        w.line(z=z)
        for r in rings:
            w.line(dome.cx + r, dome.cy)
            w.arc(True, dome.cx - r, dome.cy, -r, 0)
            w.arc(True, dome.cx + r, dome.cy, r, 0)
        w.rapid(z=10)

    w.section("dome finish")
    radius = 3.0
    w.tool(21, 2 * radius, ball_endmill(2 * radius))
    extent = dome.radius + 1.5  # the dome and the allowance left by roughing around it
    n = max(10, int(math.sqrt(n_moves / 2 * 4 / math.pi)))
    tan_lead = math.tan(math.radians(lead))
    max_tilt = math.radians(max_tilt)
    for allowance in (1.0, 0.0):
        # ball center heights of a ball bigger by the allowance, the actual ball stays the
        # allowance away from the dome
        table, step = dome.center_heights(radius + allowance, extent=extent + 1)

        def center_z(r):
            f = r / step
            i = min(int(f), len(table) - 2)
            return table[i] + (table[i + 1] - table[i]) * (f - i)

        first = True
        for row in range(n):
            y = dome.cy - extent + 2 * extent * row / (n - 1)
            xs = [dome.cx - extent + 2 * extent * i / (n - 1) for i in range(n)]
            feed = 1.0 if row % 2 == 0 else -1.0
            if feed < 0:
                xs.reverse()
            # only over the dome and its rim
            xs = [x for x in xs if math.hypot(x - dome.cx, y - dome.cy) <= extent]
            if not xs:
                continue
            for i, x in enumerate(xs):
                # normal of the surface of the ball centers, which is the contact normal
                r = math.hypot(x - dome.cx, y - dome.cy)
                slope = (center_z(r + step) - center_z(max(0.0, r - step))) / (2 * step)
                ux, uy = ((x - dome.cx) / r, (y - dome.cy) / r) if r > 1e-9 else (0.0, 0.0)
                dx, dy = slope * ux, slope * uy
                nl = math.sqrt(dx * dx + dy * dy + 1)
                normal = (-dx / nl, -dy / nl, 1 / nl)
                # feed direction in the tangent plane, the axis leans forward in it
                t = (feed, 0.0, feed * dx)
                tl = math.sqrt(t[0] * t[0] + t[2] * t[2])
                axis = [normal[k] + tan_lead * t[k] / tl for k in range(3)]
                length = math.sqrt(sum(v * v for v in axis))
                axis = [v / length for v in axis]
                if math.acos(axis[2]) > max_tilt:
                    h = math.hypot(axis[0], axis[1])
                    axis = [
                        axis[0] / h * math.sin(max_tilt),
                        axis[1] / h * math.sin(max_tilt),
                        math.cos(max_tilt),
                    ]
                # the tip is a radius from the ball center, along the axis
                center = (x, y, center_z(r))
                tip = [center[k] - axis[k] * radius for k in range(3)]
                a, b = rotary_for_axis(axis)
                if i == 0:
                    # to the start of the row above the stock, then down with its orientation
                    w.rapid(z=10 if first else 2)
                    first = False
                    w.rapid(tip[0], tip[1], a=a, b=b)
                w.line(tip[0], tip[1], tip[2], a=a, b=b)
        w.rapid(z=10)
    w.rapid(a=0, b=0)


def op_swarf_island(w, area, n_moves, draft=15.0):
    """Flank milling of a square island with drafted walls: the tool axis is parallel to the
    walls, leaning towards the island, and turns at the corners."""
    w.section("swarf island")
    radius = 4.0
    w.tool(22, 2 * radius, flat_endmill(2 * radius))
    x0, y0, x1, y1 = area
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    half = 8.0  # half the island width at the top
    tan = math.tan(math.radians(draft))
    levels = max(3, min(20, n_moves // 40))
    corners = [  # (x sign, y sign, A, B): the axis leans towards the island
        (1, -1, -draft, -draft),
        (1, 1, draft, -draft),
        (-1, 1, draft, draft),
        (-1, -1, -draft, draft),
    ]
    w.rapid(z=10)
    for level in range(1, levels + 1):
        z = -10.0 * level / levels
        # the wall at the depth of the tool tip, and the tool axis a radius away from it
        offset = half - z * tan + radius / math.cos(math.radians(draft))
        sx, sy, a, b = corners[-1]
        w.rapid(cx + sx * (offset + 5), cy + sy * (offset + 5), a=a, b=b)
        w.line(z=z)
        w.line(cx + sx * offset, cy + sy * offset)
        for sx, sy, a, b in corners:
            w.line(cx + sx * offset, cy + sy * offset, a=a, b=b)
        w.rapid(z=10)
    w.rapid(a=0, b=0)


def op_swivel_grooves(w, area, n_moves, swivel=30.0):
    """Grooves along X, the tool swivels from one side to the other along each groove: moves
    with simultaneous motion of the linear and the rotary axes."""
    w.section("swivel grooves")
    w.tool(21, 6.0, ball_endmill(6.0))
    x0, y0, x1, y1 = _shrink(area, 4.0)
    grooves = 4
    steps = max(10, n_moves // grooves)
    w.rapid(z=10)
    for g in range(grooves):
        y = y0 + (y1 - y0) * g / (grooves - 1)
        w.rapid(x0, y, a=-swivel)
        for i in range(steps + 1):
            t = i / steps
            x = x0 + (x1 - x0) * t
            z = -2.0 - 2.0 * math.sin(t * math.pi)
            w.line(x, y, z, a=-swivel + 2 * swivel * t)
        w.rapid(z=10)
    w.rapid(a=0)


def op_side_features(w, area, n_moves):
    """Holes drilled into two side faces of the stock and a slot milled into a side face, with
    the tool horizontal. The tool turns only at a safe height, away from the stock."""
    x0, y0, x1, y1 = area
    safe = 30.0
    w.section("side drilling")
    w.tool(23, 5.0, drill(5.0))
    holes = max(3, min(12, n_moves // 20))
    # into the face at x = 100: B 90 points the shank towards +X
    w.rapid(z=safe)
    w.rapid(110, y0 + 5, safe, b=90)
    for i in range(holes):
        y = y0 + 5 + (y1 - y0 - 10) * i / (holes - 1)
        w.rapid(110, y, -6)
        w.line(88, y, -6)
        w.rapid(110, y, -6)
    w.rapid(z=safe)
    # into the face at y = 0: A 90 points the shank towards -Y
    w.rapid(x0 + 5, -50, safe, a=90, b=0)
    for i in range(holes):
        x = x0 + 5 + (x1 - x0 - 10) * i / (holes - 1)
        w.rapid(x, -10, -14)
        w.line(x, 12, -14)
        w.rapid(x, -10, -14)
    w.rapid(y=-50)
    w.rapid(z=safe)
    w.rapid(a=0)

    w.section("side slot")
    w.tool(24, 6.0, flat_endmill(6.0))
    w.rapid(110, y0 + 4, safe, b=90)
    w.rapid(z=-10)
    for depth in (96.0, 92.0):
        w.line(depth, y0 + 4, -10)
        w.line(depth, y1 - 4, -10)
        w.line(110, y1 - 4, -10)
        w.rapid(110, y0 + 4, -10)
    w.rapid(z=safe)
    w.rapid(b=0)


# Jobs


def pocket(n_moves):
    """Pocket with many step down levels."""
    w = _Writer()
    op_pocket(w, FULL_AREA, n_moves, depth=15.0)
    return w.job("pocket")


def surface3d(n_moves):
    """Ball end raster finishing of a free form surface, many short 3D moves."""
    w = _Writer()
    op_surface(w, FULL_AREA, n_moves)
    return w.job("surface3d")


def adaptive(n_moves):
    """Trochoidal slotting: lots of small arcs, like adaptive clearing."""
    w = _Writer()
    op_adaptive(w, FULL_AREA, n_moves)
    return w.job("adaptive")


def drilling(n_moves):
    """Grid of canned drill cycles."""
    w = _Writer()
    w.section("drill")
    w.tool(4, 5.0, drill(5.0))
    holes = max(4, n_moves // 4)
    n = max(2, int(math.sqrt(holes)))
    w.retract()
    for i in range(n):
        for j in range(n):
            w.drill(10 + 80 * i / (n - 1), 10 + 80 * j / (n - 1), -12, 2)
    w.retract()
    return w.job("drilling")


def tool_shapes(n_moves):
    """Every tool shape FreeCAD supports, including undercut tools."""
    w = _Writer()
    op_tool_shapes(w, (25.0, 25.0, 75.0, 75.0), n_moves)
    return w.job("tool_shapes")


def mixed(n_moves):
    """A complex job: every operation in its own quarter of the stock.

    Pocket and adaptive clearing are limited to realistic step sizes, like on a real job most of
    the moves of large jobs come from 3D finishing.
    """
    w = _Writer()
    op_pocket(w, (5.0, 55.0, 45.0, 95.0), n_moves * 25 // 100)
    op_adaptive(w, (55.0, 55.0, 95.0, 95.0), n_moves * 25 // 100)
    op_tool_shapes(w, (52.0, 2.0, 98.0, 48.0), n_moves * 10 // 100)
    op_surface(w, (5.0, 5.0, 45.0, 45.0), max(n_moves * 20 // 100, n_moves - len(w.lines)))
    return w.job("mixed")


def five_axis(n_moves):
    """5 axis machining: dome finishing with the tool following the surface normal, flank milling
    with a turning tool axis, swiveling grooves and horizontal side features, every operation in
    its own quarter of the stock."""
    w = _Writer()
    op_swarf_island(w, (55.0, 55.0, 95.0, 95.0), n_moves * 5 // 100)
    op_swivel_grooves(w, (5.0, 55.0, 45.0, 95.0), n_moves * 25 // 100)
    op_side_features(w, (55.0, 5.0, 95.0, 45.0), n_moves * 5 // 100)
    op_dome(w, (5.0, 5.0, 45.0, 45.0), max(n_moves * 20 // 100, n_moves - len(w.lines)))
    return w.job("five_axis")


JOBS = {
    "pocket": pocket,
    "surface3d": surface3d,
    "adaptive": adaptive,
    "drilling": drilling,
    "tool_shapes": tool_shapes,
    "mixed": mixed,
    "five_axis": five_axis,
}


# CAM documents, by job name: the file, relative to this directory. Their job is simulated with the
# operations that are active in the document, as the simulator task panel does.
DOCUMENT_JOBS = {
    "manifold_block": "../../DemoParts/manifold_block_3plus2.fcstd",
}
# the size of a document job
DOCUMENT_SIZE = "document"


def document_path(name):
    """The file of a document job."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, DOCUMENT_JOBS[name]))


def make_job(name, size):
    if name in DOCUMENT_JOBS:
        raise ValueError(f"{name} is a document job, it is read inside FreeCAD")
    n_moves = SIZES[size] if isinstance(size, str) else int(size)
    job = JOBS[name](n_moves)
    job["size"] = size
    return job


# Analysis


def _profile_radius_table(profile, step):
    """Radius of the tool at heights 0, step, 2 * step, ... above the tip (0 = no tool)."""
    points = list(zip(profile[0::2], profile[1::2]))
    top = max(z for _, z in points)
    heights = [i * step for i in range(int(top / step) + 1)]
    table = []
    for h in heights:
        r = 0.0
        for (r1, z1), (r2, z2) in zip(points, points[1:]):
            lo, hi = min(z1, z2), max(z1, z2)
            if lo <= h <= hi:
                if hi == lo:
                    r = max(r, r1, r2)
                else:
                    r = max(r, r1 + (r2 - r1) * (h - z1) / (z2 - z1))
        table.append(r)
    return table


def _moves(job, step):
    """Yield (line index, cutting, [tool tip positions and axes]) for every motion, like the
    simulator. Points are (x, y, z, axis x, axis y, axis z)."""
    pos = [0.0, 0.0, 10.0]
    rotary = [0.0, 0.0, 0.0]
    for index, line in enumerate(job["gcode"]):
        words = {}
        for token in line.split():
            words[token[0]] = float(token[1:])
        g = int(words.get("G", -1))
        if "T" in words:
            yield index, None, int(words["T"])
            continue
        target = [words.get("X", pos[0]), words.get("Y", pos[1]), words.get("Z", pos[2])]
        new_rotary = [words.get(k, rotary[i]) for i, k in enumerate("ABC")]
        a0, a1 = tool_axis(*rotary), tool_axis(*new_rotary)
        rotary = new_rotary

        def with_axes(points):
            out = []
            for i, p in enumerate(points):
                t = (i + 1) / len(points)
                axis = [a0[k] + (a1[k] - a0[k]) * t for k in range(3)]
                length = math.sqrt(sum(v * v for v in axis))
                out.append(list(p) + [v / length for v in axis])
            return out

        if g == 81:
            r_plane = words["R"]
            path = [(target[0], target[1], r_plane), target[:], (target[0], target[1], r_plane)]
            points = []
            prev = pos
            for p in path:
                points += _line_points(prev, p, step)
                prev = p
            pos = list(path[-1])
            yield index, True, with_axes(points)
        elif g in (2, 3):
            points = _arc_points(pos, target, words["I"], words["J"], g == 2, step)
            yield index, True, with_axes(points)
            pos = target
        else:
            # rotation only moves are sampled by the motion of the tool top
            dist = max(math.dist(pos, target), 40 * math.dist(a0, a1))
            points = _line_points(pos, target, step, max(1, math.ceil(dist / step)))
            yield index, g == 1, with_axes(points)
            pos = target


def _line_points(a, b, step, n=None):
    if n is None:
        n = max(1, math.ceil(math.dist(a, b) / step))
    return [[a[k] + (b[k] - a[k]) * i / n for k in range(3)] for i in range(1, n + 1)]


def _arc_points(a, b, i, j, cw, step):
    cx, cy = a[0] + i, a[1] + j
    r = math.hypot(i, j)
    start = math.atan2(a[1] - cy, a[0] - cx)
    end = math.atan2(b[1] - cy, b[0] - cx)
    sweep = end - start
    if cw and sweep >= 0:
        sweep -= 2 * math.pi
    elif not cw and sweep <= 0:
        sweep += 2 * math.pi
    n = max(1, math.ceil(abs(sweep) * r / step))
    return [
        [
            cx + r * math.cos(start + sweep * k / n),
            cy + r * math.sin(start + sweep * k / n),
            a[2] + (b[2] - a[2]) * k / n,
        ]
        for k in range(1, n + 1)
    ]


def _cut_tilted(material, stock, cell, layer, table, radius, height, tip, axis):
    """Remove the voxels inside a tilted tool, returns their number."""
    import numpy

    x0, y0, z0 = stock[0], stock[1], stock[2]
    nx, ny, nz = material.shape
    top = [tip[k] + axis[k] * height for k in range(3)]
    lo = [min(tip[k], top[k]) - radius for k in range(3)]
    hi = [max(tip[k], top[k]) + radius for k in range(3)]
    i0, i1 = max(0, int((lo[0] - x0) / cell)), min(nx, int((hi[0] - x0) / cell) + 2)
    j0, j1 = max(0, int((lo[1] - y0) / cell)), min(ny, int((hi[1] - y0) / cell) + 2)
    k0, k1 = max(0, int((lo[2] - z0) / layer)), min(nz, int((hi[2] - z0) / layer) + 2)
    if i0 >= i1 or j0 >= j1 or k0 >= k1:
        return 0
    vx = (x0 + (numpy.arange(i0, i1) + 0.5) * cell - tip[0])[:, None, None]
    vy = (y0 + (numpy.arange(j0, j1) + 0.5) * cell - tip[1])[None, :, None]
    vz = (z0 + (numpy.arange(k0, k1) + 0.5) * layer - tip[2])[None, None, :]
    h = vx * axis[0] + vy * axis[1] + vz * axis[2]
    rho2 = vx * vx + vy * vy + vz * vz - h * h
    index = numpy.clip((h / (layer / 2)).astype(int), 0, len(table) - 1)
    inside = (h >= 0) & (h < height) & (rho2 <= table[index] ** 2)
    block = material[i0:i1, j0:j1, k0:k1]
    count = int((block & inside).sum())
    if count:
        block &= ~inside
    return count


def analyze(job, cell=0.5, layer=0.2, result=None):
    """Material removed by every operation, using a coarse voxel model of the stock.

    Returns a list of dicts per section: name, moves, cutting (fraction of the feed moves that
    remove material) and removed (mm³). The voxels (x, y, z) are stored in result["material"] if
    a result dict is given.
    """
    import numpy

    x0, y0, z0, sx, sy, sz = job["stock"]
    nx, ny, nz = int(sx / cell), int(sy / cell), int(sz / layer)
    material = numpy.ones((nx, ny, nz), dtype=bool)
    layer_z = z0 + (numpy.arange(nz) + 0.5) * layer
    tables = {
        t[0]: (numpy.array(_profile_radius_table(t[2], layer / 2)), t[2]) for t in job["tools"]
    }
    sections = sorted(job["sections"], key=lambda s: s[1])
    stats = [{"name": name, "moves": 0, "cutting": 0, "removed": 0.0} for name, _ in sections]
    starts = [start for _, start in sections]

    table = None
    radius = 0.0
    section = -1
    for index, cutting, points in _moves(job, cell / 2):
        while section + 1 < len(starts) and index >= starts[section + 1]:
            section += 1
        if cutting is None:
            table = tables[points][0]
            profile = tables[points][1]
            radius = max(profile[0::2])
            continue
        if table is None or not points:
            continue
        removed = 0
        height = (len(table) - 1) * layer / 2
        for px, py, pz, ax, ay, az in points:
            if abs(ax) > 1e-6 or abs(ay) > 1e-6:
                removed += _cut_tilted(
                    material,
                    job["stock"],
                    cell,
                    layer,
                    table,
                    radius,
                    height,
                    (px, py, pz),
                    (ax, ay, az),
                )
                continue
            i0 = max(0, int((px - radius - x0) / cell))
            i1 = min(nx, int((px + radius - x0) / cell) + 2)
            j0 = max(0, int((py - radius - y0) / cell))
            j1 = min(ny, int((py + radius - y0) / cell) + 2)
            if i0 >= i1 or j0 >= j1:
                continue
            h = layer_z - pz
            k = numpy.nonzero((h >= 0) & (h < (len(table) - 1) * layer / 2))[0]
            if len(k) == 0:
                continue
            r = table[(h[k] / (layer / 2)).astype(int)]
            xs = x0 + (numpy.arange(i0, i1) + 0.5) * cell - px
            ys = y0 + (numpy.arange(j0, j1) + 0.5) * cell - py
            d2 = xs[:, None] ** 2 + ys[None, :] ** 2
            mask = d2[:, :, None] <= (r**2)[None, None, :]
            block = material[i0:i1, j0:j1, k[0] : k[-1] + 1]
            hit = block & mask
            count = int(hit.sum())
            if count:
                removed += count
                block &= ~mask
        if section >= 0:
            s = stats[section]
            if cutting:
                s["moves"] += 1
                s["cutting"] += 1 if removed else 0
            s["removed"] += removed * cell * cell * layer
    for s in stats:
        s["cutting"] = s["cutting"] / s["moves"] if s["moves"] else 0.0
    if result is not None:
        result["material"] = material
    return stats


def _main(argv):
    if len(argv) >= 2 and argv[1] == "analyze":
        name = argv[2] if len(argv) > 2 else "mixed"
        size = argv[3] if len(argv) > 3 else "small"
        job = make_job(name, size)
        print(f"{name} {size}: {len(job['gcode'])} lines")
        print(f"{'operation':22s} {'feed moves':>10s} {'cutting':>8s} {'removed mm3':>12s}")
        for s in analyze(job):
            print(
                f"{s['name']:22s} {s['moves']:10d} {s['cutting'] * 100:7.0f}%"
                f" {s['removed']:12.0f}"
            )
        return
    for name in JOBS:
        for size in SIZES:
            job = make_job(name, size)
            print(f"{name:12s} {size:7s} {len(job['gcode']):8d} lines, {len(job['tools'])} tools")
    for name in DOCUMENT_JOBS:
        print(f"{name:12s} {DOCUMENT_SIZE:7s} {document_path(name)}")


if __name__ == "__main__":
    _main(sys.argv)
