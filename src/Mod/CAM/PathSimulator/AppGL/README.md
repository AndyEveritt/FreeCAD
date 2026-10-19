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
  sphere is inside the tool. The task panel also gives the arcs of the profiles of the tool
  solids (`toolProfile` in `Path/Main/SimulatorFeed.py`, `CAMSim.AddTool(arcs=...)`): their edges
  are made again from the circle, within 1 um and at most 32 per quarter circle, so arcs on the
  axis are exact spheres and corner radii follow their circle. The normals of frustums are interpolated between neighboring edges
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

## Problems

The simulation finds problems on the toolpath as the frontier simulates it (`Material::Event`,
`Simulation::Events`): the cut of the frontier measures the material every motion removes
(`Grid::Cut` with `CutStats`: its volume, the lengths along the lines of the three axes in the
brick of each line times their cells, averaged, and its centroid). Events of the same kind and
operation on motions following each other (up to 3 segments apart) make one event. The kinds:

- rapid into the stock: a rapid move (G0) removes more than a twentieth of the square of the
  radius of its tool (mm3), more than grazing the material.
- the tool above its flutes meets the material: the length of the flutes of the tools comes with
  them (`CAMSim.AddTool(flutes=...)`, the `CuttingEdgeHeight` of the tool bit), the part of the
  tool above it (`Tool::ProfileAbove`) grown by 0.01 mm (`ShankMargin`) probes the material before
  the motion (a probe sweep: it measures, it doesn't remove), so it finds a shank rubbing the
  walls the flutes cut as well as plunging into the material. Its flutes are below it in the
  motion, they can't remove the material it meets. More than the margin over the radius of the
  tool in mm2 is an event. Only motions where the planner finds the material higher than the
  flutes above the tip are probed.

- cut into the model, material left: once the whole path is simulated, and the model of the job
  is there (`Simulation::SetModel`, the distance model of the distance colors, made whenever the
  job has a model), the faces the toolpath cut are measured against it: points of the final stock
  at the ends of the material along the lines, about every 0.25 mm (`RegionSpacing`), cut into
  the model by more than the tolerance (`SimulatorDeviationTolerance`), or away from it by more,
  closer than the 1 mm range of the distance (farther, like the far side of a groove around the
  part, it is not the part). They are joined into regions (in cells of 1 mm, with the cells
  within 2.5 times the spacing of their points, 3 points at least), the operation that cut most
  of a region is its operation. The regions of one kind and operation make one event, at the
  deepest one, with their number and box, at the end of the operation on the timeline: a finish
  leaving scallops above the tolerance is one problem. Changing the tolerance or the model finds
  them again.

Without a view: `ToolpathCheck` simulates a toolpath, given as the simulator is given it, and
returns its problems (`CAMSimulator.checkToolpath(steps, stock, model, quality, tolerance)`, the
steps of `simulationSteps` in `Path/Main/SimulatorFeed.py`). The Sanity Check report lists them as
squawks of their operation (`Sanity._simulationSquawks`): rapids into the stock CAUTION, the tool
above its flutes and cuts into the model WARNING, material left NOTE.

The view marks the events on the timeline in the color of their kind, with buttons to go to the
previous and the next one; the task panel lists them (`CAMSim.GetEvents`,
`SetEventsChangedCallback`), a click goes to the start of one (`CAMSim.GoToEvent`). Seeking and
scrubbing cut the view grid without measuring, they cost nothing more.

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
  lights are fixed to the view: a key light from the upper left, a fill light from the right, light
  from above brighter than from below and a highlight of the key light, so faces turned differently
  differ in brightness; the opaque objects get dark outlines (optional) where neighboring pixels
  fold by more than about 40 degrees, step far off the plane of each other, and at the background.
  The
  toolpath is drawn a few pixels closer to the camera than it is, so a path on the surface of the
  stock, like the bottom of a ball end mill groove, is not hidden by it. A section across an axis cuts the stock and the model
  (`SimDisplay::SetSection`): their shaders discard the side of the plane, and their caps are drawn
  where the plane is inside them: drawn again into the stencil only, inverted at every surface,
  odd where the plane is inside, a quad on the plane is drawn there (the stencil method; the model's
  cap after the stock's, at the same depth it shows in it). The stock's cap is picked as uncut
  stock. Rapid moves (G0) are drawn in the
  rapid color of the preferences (`DefaultRapidPathColor`), faint once passed. The path is drawn
  as separate lines (`MillPathLine`), each with the segment and the rapid flag of its end. The
  origin of the job is drawn
  as its axes, like the toolpath, and seen through the objects in front of it. The model is drawn a
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

The task panel lists the commands the simulator runs (`CommandListModel`, made as shown, in the
units of the preferences for the size of the stock, the tool directions only if the tool turns):
the command in progress is selected as the simulation goes (`CAMSim.SetCommandChangedCallback`),
a click on a command goes to its start (`CAMSim.GoToCommand`), the arrows to the next command
that moves the tool or to the start of the command in progress or the previous one
(`CAMSim.StepCommand`). Motions know the command they come from (`MillMotion::source`).

Stock to document (task panel) adds the stock shown, at the current position of the simulation,
to the document of the job as a mesh (`CAMSim.GetStockMesh`: the meshes of the mesh units joined
at their equal vertices, closed): for the stock of a next setup or rest machining, to compare it
with the model (Mesh Design, Inspection), or a solid from it (Part, Create shape from mesh).

The display options of the task panel are the parameters below, the simulator follows their
changes (`CAMSettings`): the colors of the stock, the thin wall threshold and the model tolerance,
and the stock and the model shown, with their opacities. The colors are plain, by operation (the
path color of an operation where it has been set, otherwise a color of a palette, shown in the task
panel), thin walls in red, or the distance from the model: within the tolerance green, where
material is left blue, where the stock is cut into the model red, darker up to 10 tolerances away,
with a scale at the top right of the view.
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

## Toolpath

The task panel gives the simulator the commands of the toolpath (`CAMSim.AddCommand`), as
`Path/Main/SimulatorFeed.py` prepares them: in world coordinates, the tool direction in A, B, C,
drill cycles expanded, arcs of tilted work planes divided into lines. The commands reach
`GCodeParser` as they are, their name and their words in double precision, without going through
G-code text; G-code text (`AddLine`, `Parse`) is read the same way, with its feed rates per minute.
The parser makes the motions (`MillMotion`): positions, rapid (G0) or not, the feed rate (F, mm/s,
modal), the tool, the operation and the index of the command they come from (`source`), numbered in
the order the commands were added. Motion modes, the arc plane (G17 to G19), absolute and
incremental positions (G90, G91) and the units (G20, G21) are modal; drill cycles are the moves of
the tool (to the hole, rapid to R, down to Z, back to the initial height or R); arcs are given by
their center or their radius (R), arcs outside the XY plane become lines within 1 um; comments in
parentheses are skipped within a line; home and offset commands (G28, G30, G53, G10, G92) don't
move. Where the tool is before its first motions is unknown: they take the axes they don't name
from the first motion that names them (`FillStart`, and `SimulatorFeed` for the first
operation), so the tool starts where the program takes it, not at the origin.

## Time

The timeline is in the time of the machine (`MotionSeconds`): feed moves at their programmed
feed rate (modal, also across operations, as on the machine), else at the feed rate of their tool
controller (the vertical one for moves more vertical than horizontal), rapid moves at the rapid
rates of their tool controller, else at `SimulatorRapidRate`, the axes at once: the longer of the
horizontal and the vertical. Turns of the tool take at least their largest angle at 30 degrees a
second feeding, 90 rapid. Accelerations are not modeled. A step is a millisecond, every motion has
at least one (`MillSimulation::StepsPerSecond`); the playback speed is in times the time of the
machine (x1 to x1000, x10 at first). The view shows the time at the tool and in all, the task
panel the cycle time and the time of every operation (`CAMSim.GetTimes`). The stencil renderer
has steps of its own, by the length of the motions (`ApplyTimeline` when it is switched).

## 5 axis toolpaths

`GCodeParser` reads the rotary axes A, B and C. `MotionToolAxis` in `MillMotion.h` turns them into
the tool axis: positions are tool tip positions in workpiece coordinates (tool center point
programming), and the tool axis is the Z axis rotated by A around X, then B around Y, then C
around Z. This is the place to add machine specific kinematics.

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
- `SimulatorOutlines` (bool, default true): dark lines at the edges of the opaque objects.
- `SimulatorOriginVisible` (bool, default true): the origin of the job and its axes (X red, Y
  green, Z blue, a fifth of the size of the stock long), seen through the objects in front of it.
- `SimulatorInspect` (int, default 0): lines over the stock to inspect it: 0 none, 1 the mesh, 2 the
  normals, 3 the dexels at a click, 4 measure: a click picks a point of any surface (the G-buffer
  position and normal at the pixel), two give their distance, shown at the top left of the view
  in the units of the preferences, with the thickness of the stock behind the first point along
  its normal (a ray through the mesh of the stock shown, `View::RayDistance`).
- `SimulatorStockOpacity`, `SimulatorModelOpacity` (float, 0 to 1, default 1): opacity of the
  stock (not with the stencil renderer) and of the model.
- `SimulatorSection` (int, default 0): a section of the stock and the model, 0 none, 1 across X,
  2 across Y, 3 across Z, at `SimulatorSectionPosition` (float, 0 to 1 across the box of the stock,
  default 0.5), the side below kept, the side above with `SimulatorSectionFlip` (bool).
- `SimulatorRapidRate` (float, mm/min, default 5000): the rapid rate of tool controllers
  without one, for the time of the machine.
- `SanitySimulation` (bool, default true): the Sanity Check report simulates the job and lists the
  problems found, at `SanitySimulationQuality` (int, 1 to 10, default 5).

## Tests and benchmark

- Unit tests (`CAM_tests_run`): of the material simulation in
  `tests/src/Mod/CAM/App/MaterialSimulator.cpp` (sweeps of all tool shapes, fixed, turning and
  along arcs, against sampled tools; exact cuts; seeks against simulating from the start; a closed
  mesh, also after partial updates; thin walls; the operations of cut faces), its accuracy
  against the exact material of small jobs in `MaterialAccuracy.cpp`, and the toolpath read from
  commands and G-code text in `SimulatorGCode.cpp`.
- Accuracy validation: `tests/src/Mod/CAM/App/SimulatorAccuracy.h`, an oracle of the exact material
  of a job and the measurement of a stock mesh against it, and the tool `CAM_simulator_accuracy`
  that runs it on jobs written by the simulator. See `../Benchmark/README.md`.
- Performance, image regression and accuracy benchmark: `../Benchmark/README.md`.
- With the environment variable `CAM_SIM_DUMP` set to a file name, the simulator writes its input
  there when it starts simulating (`MaterialJob`: tools, toolpath, stock), to replay the job
  outside of the GUI. With `CAM_SIM_LOG` set to a file name, the material simulation
  appends the times of its plan, stock and simulation there when it reaches the end of the path.
