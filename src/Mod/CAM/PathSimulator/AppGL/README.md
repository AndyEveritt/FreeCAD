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
  lines parallel to the axis, and the radius of its cross sections.
- `DexelLayout`: adaptive resolution. The stock is covered by a grid of bricks of 8 x 8 x 8
  cells, which are divided into 8 bricks with cells of half the size where tools cut. Before
  simulating, the path of every tool is refined to cells of 0.5 (lowest quality) to 0.1 (highest)
  times the size of the features it cuts, so the detail follows the tools and doesn't depend on
  the size of the stock. The finest level fits the smallest tool, the stock away from the tools
  has 32 to 104 cells along its longest side. Neighboring bricks differ by at most one level.
- `DexelStock`: the dexels of the bricks. The lines of a brick extend one cell beyond it, so its
  border is known exactly in each brick. Bricks are shared by copies of the stock and copied when
  they change, so snapshots are cheap; completely empty or filled bricks hold no lines. Cuts are
  applied in parallel, every thread owning a subset of the bricks, and the material of a line
  only depends on its position, so the copies of a line in neighboring bricks stay equal.
  - vertical tools: lines along Z remove the tool solid of the closest approach of moves divided
    into pieces of at most a quarter cell of height, lines along X and Y the cross sections of
    the tool, exactly (convex hull of the sections at the profile vertices)
  - tilted or turning tools (5 axis): tool instances along the move, at most half a cell apart,
    limited to the part of the tool below the top of the stock. Where a sharp edge of a tool
    makes the surface, its normal is not known, the mesher estimates it.
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
  connect without cracks. Flat faces along the axes are merged into rectangles. Faces get the cut
  color from their crossings. The surface of every brick is kept until it changes.
- `DexelRenderer`: builds the mesh per mesh unit of the layout (up to 16 bricks), rebuilding the
  units of changed bricks within a time budget per frame, and draws it per root brick.
- `SimDisplay`: deferred shading with optional ambient occlusion (SSAO, at half resolution).

Rotating or zooming the view only draws the meshes, independent of the toolpath length. The time
budgets for simulating and meshing shrink while frames take longer than 30 ms, so heavy scenes
keep their frame rate and take a few more frames to show the current state. The view is only
rendered when something changed.

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
