# CAM simulator

OpenGL simulator of the CAM workbench (`CAM_SimulatorGL`). It removes material from the stock
along the toolpath and shows the result in its own 3D view.

## Material removal

The stock is a grid of vertical dexels (Z-dexels): lines through the cell centers, each holding the
height intervals where there is material. This represents any stock and any material removal,
including undercuts (dovetail, slitting saw, thread mill) and cuts with tilted tools. The detail
across the grid is the cell size, along Z it is exact.

- `CutterProfile`: a tool as the solid of revolution of its profile. Gives the material of the tool
  along a line: exactly for any line (tilted tools), and from a table for lines parallel to the axis.
- `DexelStock`: the dexel grid, in tiles of 16 x 16 cells. Tiles are shared by copies of the stock
  and copied when they change, so snapshots are cheap. Cuts are applied in parallel, every thread
  owning a subset of the tiles; cutting is order independent, so no locking is needed.
  - vertical tools: a linear move removes, at every dexel, the tool solid at its closest approach,
    over the height range of the move, split into pieces of at most a quarter cell of height
  - tilted or turning tools (5 axis): tool instances along the move, at most half a cell apart,
    limited to the part of the tool below the top of the stock
- `DexelSimulation`: maps the simulation timeline (steps of the toolpath segments) to cuts. A
  background thread simulates the whole toolpath once and keeps snapshots of the stock at regular
  amounts of work, with low thread priority. Moving to any step restores the closest snapshot
  before it and cuts the rest, within a time budget per frame; `MillSimulation::IsRenderPending`
  tells if the view is not up to date yet.

## Display

- `DexelMesher`: builds triangles from the dexels, per chunk of 32 x 32 cells. Neighboring cell
  tops within 5 cells of height form a smooth surface with shared vertices and normals; side faces
  only separate tops that are not on one surface. Flat faces and straight walls are merged. Faces
  not on the surface of the initial stock are cut faces, drawn in the cut color.
- `DexelRenderer`: keeps the chunks in vertex buffers and rebuilds the chunks of changed tiles,
  within a time budget per frame.
- `SimDisplay`: deferred shading with optional ambient occlusion (SSAO, at half resolution).

Rotating or zooming the view only draws the chunk meshes, independent of the toolpath length. The
time budgets for simulating and meshing shrink while frames take longer than 30 ms, so heavy scenes
keep their frame rate and take a few more frames to show the current state. The view is only
rendered when something changed.

The cell size follows the accuracy of the simulator task panel: 138 (lowest) to 480 (highest)
cells along the larger side of the stock. Cells are at most 0.75 times the radius of the smallest
tool, so small tools cut on large stock, but not more than 1200 along the larger side.

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

- Unit tests of the material removal: `tests/src/Mod/CAM/App/Simulator.cpp` (`CAM_tests_run`).
- Performance and image regression benchmark: `../Benchmark/README.md`.
