# CAM simulator

OpenGL simulator of the CAM workbench (`CAM_SimulatorGL`). It removes material from the stock
along the toolpath and shows the result in its own 3D view.

The material simulation (`Material*`) does it by default. The original stencil renderer is still
there too (`SimulatorStencilRendering`).

## Material removal

The stock is stored as tri-dexels: lines along X, Y and Z through the points of a grid, each
holding the ranges where there is material. Every end of a range is a point of the surface,
exact along its line, with the normal of the surface there, whether it was cut and by which
operation (packed into 32 bits: the normal in 20, within about 0.1 degrees, the operation in 10,
up to 1023 operations, see `PackNormal`). This represents any stock and any material removal,
including undercuts (dovetail, slitting saw, thread mill) and cuts with tilted and turning tools.

- `MaterialTool`: a tool as the solid of revolution of its profile, made of primitives whose
  sweeps along a straight motion meet every line in one interval: spheres, the arcs of the
  profile centered on the axis (ball ends, the tips of engravers), and frustums, the straight
  edges (cylinders and cones). An arc given as edges is fitted by a sphere if the rest of the
  sphere is inside the tool. The normals of frustums are interpolated between neighboring edges
  where the profile is smooth, so a bull nose corner given as edges cuts smoothly shaded surfaces.
  Probes (all of their solid below the tip) don't cut. `Sweep` intersects the volume a tool sweeps
  along a motion with lines, exactly:
  - spheres: a capsule along the motion, in closed form for any line.
  - frustums with a fixed axis (3 axis, 3+2): the points of a line inside a frustum at some time of
    the motion form a convex region of the plane of the line position and the time, bounded by a
    conic and straight lines; its extremes are where the lines meet the conic, at their corners,
    or where the conic is tangent to the lines of constant position, found in closed form.
  - motions across the tool axis, the most common: lines along the axis from the closest approach
    of the motion, by the primitives sorted by height; lines across the axis by the largest cross
    section at their height swept along the motion.
  - arcs of tools along Z at a constant height (pockets, adaptive clearing): exactly, in pieces of
    up to a quarter circle, lines along the axis by their distance from the arc, lines across it
    by the cross section swept along the arc (an annulus in the wedge of the arc and the disks at
    its ends). Other arcs are divided into chords.
  - turning tools (5 axis): every primitive in its own pieces, turning about its center by a small
    angle while its center moves on a chord of its path, within a fiftieth of the cells of the
    tool of the exact sweep. A sphere doesn't change turning about its center, so few pieces sweep
    a ball end exactly; long frustums are split. The pieces of a primitive are culled together, by
    the sweep of the primitive grown by their error.
  - the tool beyond the box of the stock along its axis, like the shank above it, is left out
    (`CuttingHeight`).
  The ends of the removals carry how to find their normal: on the tool where the motion reaches
  them, unknown at sharp edges of the tool.
- `MaterialLayout`: adaptive resolution, an octree of bricks of 8 x 8 x 8 cells over a grid of
  root bricks, neighboring bricks at most one level apart. It is built from the bricks requested
  at every level, collected on all threads and counted once, in one pass from the finest level up.
- `MaterialGrid`: the dexels of the bricks. The lines of a brick extend one cell beyond it, so the
  material crossing its border has no ends there. Bricks are shared by copies of the grid and
  replaced when cut, so checkpoints are cheap; empty and completely filled bricks hold no lines.
  The stock mesh fills the bricks near its triangles, the others are all inside or all outside.
  A batch of motions is cut brick by brick, on all threads: every brick once, with all motions
  touching it in their order, so later motions skip the lines earlier ones emptied. A brick only
  gets the motions whose swept volume can reach it, and every motion only the lines of the brick
  in its box. Normals are only computed for the ends that remain.
- `MaterialSimulation`: the plan and the timeline, in a worker thread with low priority.
  - plan: the path divided into motions; the cells of every tool, a fraction of the features it
    cuts (0.5 of them at the lowest quality, 0.1 at the highest), where it can reach the material:
    the material top along tool axes along the axes of the stock is followed on height maps,
    lowered where tools without undercuts leave nothing above their bottom, so a small tool
    finishing below material removed by a large one refines little. The stock away from the tools
    has 32 to 256 cells along its longest side. Grooves: where passes of a ball end run beside
    each other (found by the paths of the ball centers, in 3D) and leave ridges steeper than 10
    degrees and higher than a tenth of the cells of the tool, the cells along the passes get an
    eighth of their distance, up to 2 levels finer than the tool at the highest quality.
  - timeline: the worker simulates the whole path once (the frontier), with a checkpoint at every
    40 ms of cutting, at most 768 MB of them (every other one is dropped beyond). A step before the
    frontier is reached from the closest checkpoint before it, a motion in progress is cut into
    the stock shown only. The stock shown is published to the display when it is at the requested
    step, or every 400 ms on the way to it while the frontier catches up (meshing and drawing
    every state would take the time from the simulation).

Arcs and turning motions are approximated within a fiftieth of the cells of the features of their
tool, far below what the cells show. Axes of work planes along the axes of the stock, computed
from angles, are snapped to them.

## Display

- `MaterialMesher`: dual contouring. Grid points are inside the material if the Z dexel through them
  is, its ends included (like the material a tool only touches); the edges between grid points lie
  on dexels, which have the exact points of the surface on them and its normals. Cells of a brick
  are merged into cubes of 2, 4 or 8 cells where the surface is flat (normals within about 8
  degrees, points within a tenth of a cell of a plane), or into cubes of up to 4 cells holding one
  sharp edge or corner of planes, or a ridge between curved faces (like between passes of a ball
  end), with one sheet of surface. Every cube with a surface gets a vertex: on the edge or corner of
  the planes of its points at sharp features (not of planes facing almost opposite ways, like the
  faces of a thin wall, where they meet depends too much on their normals), else where the planes of
  its points meet (every plane counting the same) nearest to their mass point, or to the points
  without a normal if they are on the planes (made by a sharp edge of a tool, on the crease it cut,
  like the tip of a V groove narrower than a cell), moved onto curved surfaces by a quadric fit.
  Every edge of the cubes with a change of inside gets a quad of the vertices of the 4 cubes around
  it, from the smallest cubes around it, so bricks of different levels connect without cracks, split
  along the diagonal passing nearest to the surface point of the edge. Vertices on smooth surfaces
  share their normal, at sharp edges (normals far apart, or in two groups, like a face and a
  chamfer) every quad gets the normal of its own surface point, or of its own plane where the point
  has none. Faces get the operation that cut them, and while thin walls are shown, the thickness of
  the material behind them: the length of the material along the dexel of their edge, followed into
  the bricks ahead, times the component of their normal along it (exact for walls with flat faces,
  up to 10 mm). The cubes of every brick are kept until it changes.
- `MaterialRenderer` (`Material::View`): the simulation, a mesher thread and the GPU buffers. The
  mesher builds the meshes of the stock the simulation publishes, for the mesh units (up to 8
  bricks) depending on its changed bricks; the GUI thread only uploads them, all units of a stock
  at once, so the mesh never has cracks between new and old parts, and draws them in chunks of
  units. The view asks for a frame when a new mesh is ready.
- `SimDisplay`: deferred shading with optional ambient occlusion (SSAO, at half resolution). The
  toolpath is drawn a few pixels closer to the camera than it is, so a path on the surface of the
  stock, like the bottom of a ball end mill groove, is not hidden by it. The model is drawn a
  little behind the stock: where the stock is cut to the model, the stock and the colors of its
  operations are shown, the model only where the stock is cut deeper. Transparent objects (the
  stock or the model with an opacity below 1) are drawn after the opaque ones in 4 layers (depth
  peeling): every layer draws both sides of their faces into a second G-buffer starting with the
  depth of the opaque ones, and keeps the surfaces behind those of the layer before. The layers
  are lit and blended over the opaque objects from the back, so the model shows inside a
  transparent stock, and the far side of its walls and holes.

Rotating or zooming the view only draws the meshes, independent of the toolpath length, and
nothing of the simulation or the meshing runs in the GUI thread, so the view stays responsive
while simulating. During playback the tool waits for the stock: the timeline doesn't advance
while the shown stock is behind the tool. The view is only rendered when something changed.

## Simulator window

The task panel (`Path/Main/Gui/SimulatorGL.py`) passes the operations to the simulator
(`CAMSim.AddOperation`) with their colors. The view shows the operation in progress, with buttons
to go to the start and the end, and to the start of the next or the current or previous operation.
A click on an operation in the task panel, or on a face of the stock in the view (not a drag),
goes to the start of the operation (`CAMSim.GoToOperation`) and selects it in the task panel: for
a face, the operation that cut it (`CAMSim.SetOperationPickedCallback`). The stock shader writes the
operation tag of its faces with their position in the G-buffer, read at the pixel after the next
frame (`SimDisplay::RequestPick`), from the front-most transparent layer with the stock, else the
opaque objects.

The display options of the task panel are the parameters below, the simulator follows their
changes (`CAMSettings`): the colors of the stock, the thin wall threshold and the model tolerance,
and the stock and the model shown, with their opacities. The colors are plain, by operation (the
path color of an operation where it has been set, otherwise a color of a palette, shown in the task
panel), thin walls in red, or the distance from the model: within the tolerance green, where
material is left blue, where the stock is cut into the model red, darker up to 10 tolerances away.
The distance of every vertex of the mesh is measured in the mesher thread while it is shown, from
the model tessellated within 2 um in another thread (`MaterialModel`, `ModelDistance`: its
triangles in a grid of the range of 1 mm, the sign by the normals of the nearest triangles, by the
parity of a ray beyond the range).

Inspect draws lines over the stock (`Material::View::RenderInspection`, a little towards the camera
so they show on the faces they lie on): the edges of the triangles of its mesh, the normals of its
vertices, or its dexels in the bricks around a point of the stock clicked in the view, from the
stock shown: the material along them blue, their ends as small crosses, green with a normal, red
without (made by a sharp edge of a tool), yellow where not cut. While the dexels are inspected, a
click chooses where, it doesn't go to the operation; a lower stock opacity shows the dexels inside
the material.

## 5 axis toolpaths

`GCodeParser` reads the rotary axes A, B and C. `MotionToolAxis` in `MillMotion.h` turns them into
the tool axis: positions are tool tip positions in workpiece coordinates (tool center point
programming), and the tool axis is the Z axis rotated by A around X, then B around Y, then C
around Z. This is the place to add machine specific kinematics. Arcs are in the XY plane.

## Parameters

`User parameter:BaseApp/Preferences/Mod/CAM`:

- `SimulatorStencilRendering` (bool, default false): use the original renderer, which subtracts the
  tool sweeps from the stock in screen space with the stencil buffer, instead of the simulations.
  It supports 3 axis toolpaths only, and every change of the view renders all cuts, so it gets slow
  on long toolpaths.
- `SimulatorColorMode` (int, default 1): the colors of the stock: 0 plain, 1 by operation, 2 thin
  walls, faces in front of material thinner than `SimulatorThinWallThreshold` (float, mm, default
  0.5) in red, 3 the distance from the model, within `SimulatorDeviationTolerance` (float, mm,
  default 0.05) green.
- `SimulatorStockVisible` (bool, default true), `SimulatorModelVisible` (bool, default false): the
  stock and the model shown.
- `SimulatorInspect` (int, default 0): lines over the stock to inspect it: 0 none, 1 the mesh, 2 the
  normals, 3 the dexels at a click.
- `SimulatorStockOpacity`, `SimulatorModelOpacity` (float, 0 to 1, default 1): opacity of the
  stock (not with the stencil renderer) and of the model.

## Tests and benchmark

- Unit tests (`CAM_tests_run`): of the material simulation in
  `tests/src/Mod/CAM/App/MaterialSimulator.cpp` (sweeps of all tool shapes, fixed, turning and
  along arcs, against sampled tools; exact cuts; seeks against simulating from the start; a closed
  mesh, also after partial updates; thin walls; the operations of cut faces), and its accuracy
  against the exact material of small jobs in `MaterialAccuracy.cpp`.
- Accuracy validation: `tests/src/Mod/CAM/App/SimulatorAccuracy.h`, an oracle of the exact material
  of a job and the measurement of a stock mesh against it, and the tool `CAM_simulator_accuracy`
  that runs it on jobs written by the simulator. See `../Benchmark/README.md`.
- Performance, image regression and accuracy benchmark: `../Benchmark/README.md`.
- With the environment variable `CAM_SIM_DUMP` set to a file name, the simulator writes its input
  there when it starts simulating (`MaterialJob`: tools, toolpath, stock), to replay the job
  outside of the GUI. With `CAM_SIM_LOG` set to a file name, the material simulation
  appends the times of its plan, stock and simulation there when it reaches the end of the path.
