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

    python3 jobs.py                      list all jobs and their sizes
    python3 jobs.py analyze mixed small  material removed by every operation of a job
"""

import math
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

    def rapid(self, x=None, y=None, z=None):
        self.lines.append("G0" + self._xyz(x, y, z))

    def line(self, x=None, y=None, z=None):
        self.lines.append("G1" + self._xyz(x, y, z))

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


JOBS = {
    "pocket": pocket,
    "surface3d": surface3d,
    "adaptive": adaptive,
    "drilling": drilling,
    "tool_shapes": tool_shapes,
    "mixed": mixed,
}


def make_job(name, size):
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
    """Yield (line index, cutting, [tool tip positions]) for every motion, like the simulator."""
    pos = [0.0, 0.0, 10.0]
    for index, line in enumerate(job["gcode"]):
        words = {}
        for token in line.split():
            words[token[0]] = float(token[1:])
        g = int(words.get("G", -1))
        if "T" in words:
            yield index, None, int(words["T"])
            continue
        target = [words.get("X", pos[0]), words.get("Y", pos[1]), words.get("Z", pos[2])]
        if g == 81:
            r_plane = words["R"]
            path = [(target[0], target[1], r_plane), target[:], (target[0], target[1], r_plane)]
            points = []
            prev = pos
            for p in path:
                points += _line_points(prev, p, step)
                prev = p
            pos = list(path[-1])
            yield index, True, points
        elif g in (2, 3):
            yield index, True, _arc_points(pos, target, words["I"], words["J"], g == 2, step)
            pos = target
        else:
            yield index, g == 1, _line_points(pos, target, step)
            pos = target


def _line_points(a, b, step):
    length = math.dist(a, b)
    n = max(1, math.ceil(length / step))
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


def analyze(job, cell=0.5, layer=0.2):
    """Material removed by every operation, using a coarse voxel model of the stock.

    Returns a list of dicts per section: name, moves, cutting (fraction of the feed moves that
    remove material) and removed (mm³).
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
        for px, py, pz in points:
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


if __name__ == "__main__":
    _main(sys.argv)
