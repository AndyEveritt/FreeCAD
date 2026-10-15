# CAM simulator

OpenGL simulator of the CAM workbench (`CAM_SimulatorGL`). It removes material from the stock
along the toolpath and shows the result in its own 3D view.

## Material removal

The stock is stored as tri-dexels: lines along X, Y and Z through the points of a grid, each
holding the ranges where there is material. Every end of a range is a point of the surface,
exact along its line, with the normal of the surface there and whether it was cut. This
represents any stock and any material removal, including undercuts (dovetail, slitting saw,
thread mill) and cuts with tilted tools.

- `CutterProfile`: a tool as the solid of revolution of its profile. Gives the material of the tool
  along a line with the surface normals: exactly for any line (tilted tools), from a table for
  lines parallel to the axis, and the radius of its cross sections. The normals are interpolated
  between the edges of the profile where it is smooth, so a ball approximated by edges cuts
  surfaces shaded like a real ball.
- `DexelLayout`: adaptive resolution. The stock is covered by a grid of bricks of 8 x 8 x 8
  cells, which are divided into 8 bricks with cells of half the size where tools cut. Before
  simulating, the path of every tool is refined to cells of 0.5 (lowest quality) to 0.1 (highest)
  times the size of the features it cuts, so the detail doesn't depend on the size of the stock.
  The stock away from the tools has 32 to 104 cells along its longest side. Neighboring bricks
  differ by at most one level. The refinements that follow the toolpath
  (`DexelSimulation::BuildLayout`, on all cores):
  - grooves: where passes of a tool run beside each other closer than its cells can resolve,
    like a ball end mill finishing with a small stepover, a band along the pass gets cells of
    about a quarter of the spacing (at the highest quality, up to two levels finer than the
    tool). Passes are found by the centers of the lower ends of the tools (the ball center), in
    3D, with any tool axis. Whether they leave visible grooves comes from the cross section the
    tool sweeps (`CutterProfile::SweptLowerEdge`, for any tool shape and axis): two passes leave
    grooves where the surfaces they make meet at more than 10 degrees at the ridge between them,
    so a flat end mill on a floor doesn't, tilted it does. The band follows the point where the
    tool touches the surface, also on slopes and with tilted tools, half way to the passes beside
    it. Grooves along a short path only are ignored.
  - final surface: the lower surface of the vertical tools is followed on a grid in XY, the lowest
    of all their moves. Passes that deeper moves cut away everywhere along them, like the step
    downs above the last one, get no finer cells for their grooves: they are not seen at the end.
  - levels: cells are powers of two apart, aligned to the cells of the tool cutting the longest
    path, so it gets the cells it asks for.
  - material: the top of the material is followed on a grid in XY, lowered where vertical tools
    leave nothing above their lower surface. Moves are refined only up to the material left by
    the moves before them, not to the top of the stock, so a small tool finishing below material
    removed by a large one refines little.
- `DexelStock`: the dexels of the bricks. The lines of a brick extend one cell beyond it, so its
  border is known exactly in each brick. Bricks are shared by copies of the stock and copied when
  they change, so snapshots are cheap; completely empty or filled bricks hold no lines. A group of
  moves is cut brick by brick: every brick once, with all moves touching it, divided into parts
  by axis and lines that the threads take as they become free. The material of a line only
  depends on its position, so the copies of a line in neighboring bricks stay equal.
  - vertical tools: lines along Z remove the tool solid of the closest approach of moves divided
    into pieces of at most a quarter cell of height, lines along X and Y the cross sections of
    the tool, exactly (convex hull of the sections at the profile vertices)
  - tilted or turning tools (5 axis): tool instances along the move, limited to the part of the
    tool below the top of the stock. Tools with sharp edges there have instances at most half a
    cell apart, smooth tools (ball, bull nose) farther, as long as the ridges between instances
    stay below a hundredth of a cell (`CutterProfile::SmoothRadius`). Where a sharp edge of a tool
    makes the surface, its normal is not known, the mesher estimates it.
  - Moves only visit the dexels the tool can reach: lines farther than its cutting radius (of the
    part below the top of the stock, not a wider shank), and lines whose material is below the
    lowest point of the tool at their distance from the move, like under the passes beside the
    one finishing the surface there, are skipped. Normals are only computed for the ends of a
    removal that become ends of the material.
  - Tools whose profile has tiny steps (up to a hundredth of the radius, like where the flutes meet
    the shank) still use the fast cut of vertical tools.
- `DexelSimulation`: builds the layout and maps the simulation timeline (steps of the toolpath
  segments) to cuts. A background thread simulates the whole toolpath once and keeps snapshots of
  the stock at regular amounts of work, with low thread priority. Moving to any step restores the
  closest snapshot before it and cuts the rest, within a time budget per frame;
  `MillSimulation::IsRenderPending` tells if the view is not up to date yet.

## Display

- `DexelMesher`: dual contouring. Grid points are inside the material if the Z dexel through them
  is. Cells of a brick are merged into cubes of 2, 4 or 8 cells where one vertex fits the surface
  within a tenth of a cell, so flat and gently curved surfaces get few large polygons. Every cube
  with a surface gets a vertex where the planes of the dexel crossings on its edges meet, so sharp
  edges and corners are kept. Every edge with a change of inside gets a quad of the 4 cubes around
  it, taken at the smallest cubes around the edge, so bricks and cubes of different sizes
  connect without cracks. Cubes are only merged where the normals of their crossings agree within
  about 8 degrees, so shallow features like scallops keep their shading. Ridges of cut surfaces,
  like between the passes of a ball end mill, are found where the surface bends convex between
  neighboring dexels much more than next to them: the crossings on each side of a ridge get their
  own vertex normals, so it stays sharp. Flat faces along the axes are merged into rectangles.
  Faces get the cut color from their crossings. The surface of every brick is kept until it
  changes.
- `DexelMeshUnits`: the mesh per mesh unit of the layout (up to 16 bricks). Changed bricks are
  updated oldest first within a time budget per frame, every group of touching changed bricks
  together with all units depending on them, so the mesh never has cracks between new and old
  parts.
- `DexelRenderer`: draws the mesh units per root brick.
- `SimDisplay`: deferred shading with optional ambient occlusion (SSAO, at half resolution). The
  toolpath is drawn a few pixels closer to the camera than it is, so a path on the surface of the
  stock, like the bottom of a ball end mill groove, is not hidden by it.

Rotating or zooming the view only draws the meshes, independent of the toolpath length. Every
frame has a time budget for simulating and meshing: the simulation takes what it needs, the mesh
the rest; while catching up with a far position, intermediate states are only shown after a
quarter second. The budget shrinks while frames take longer than 30 ms, so heavy scenes keep
their frame rate. During playback the tool waits for the stock: the timeline doesn't advance
while the shown stock is behind the tool. The view is only rendered when something changed.

## 5 axis toolpaths

`GCodeParser` reads the rotary axes A, B and C. `MotionToolAxis` in `MillMotion.h` turns them into
the tool axis: positions are tool tip positions in workpiece coordinates (tool center point
programming), and the tool axis is the Z axis rotated by A around X, then B around Y, then C
around Z. This is the place to add machine specific kinematics. Arcs are in the XY plane.

## Parameters

`User parameter:BaseApp/Preferences/Mod/CAM`:

- `SimulatorStencilRendering` (bool, default false): use the original renderer, which subtracts the
  tool sweeps from the stock in screen space with the stencil buffer, instead of the dexel
  simulation. It supports 3 axis toolpaths only, and every change of the view renders all cuts, so
  it gets slow on long toolpaths.

## Tests and benchmark

- Unit tests of the material removal, the layout and the mesh: `tests/src/Mod/CAM/App/Simulator.cpp`
  (`CAM_tests_run`).
- Performance and image regression benchmark: `../Benchmark/README.md`.
