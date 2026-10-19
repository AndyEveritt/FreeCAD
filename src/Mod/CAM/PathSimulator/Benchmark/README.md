# CAM simulator benchmark

Measures how responsive the CAM simulator (`CAMSimulator`, the OpenGL simulator) is on synthetic
jobs of increasing complexity, and checks that it still shows the right result: by images, and
with `--accuracy` against the exact material of every job. It is meant to be
run often during development, by developers and AI agents, and in CI. It is not installed with
FreeCAD.

## Files

- `jobs.py`: generates G-code jobs, tool profiles and stock. Uses only the Python standard library.
  Also lists the document jobs, CAM jobs of FreeCAD documents.
- `sim_benchmark.py`: runs inside the FreeCAD GUI, drives the simulator and records results.
- `run_benchmark.py`: launcher. Starts FreeCAD, prints a summary, compares with a baseline and
  reports a verdict.
- `reference/`: correctness reference images: `mixed` at sizes `small` (quick mode) and `medium`,
  `five_axis` at size `small` (quick mode), and `manifold_block` at `--quality 6`, each colored by
  operation (the default) and plain.

## Running

```sh
B=src/Mod/CAM/PathSimulator/Benchmark/run_benchmark.py

# record a baseline on this machine, before changing the simulator
python3 $B --freecad build/bin/FreeCAD --label before -o baseline.json

# after a change: run, compare with the baseline, exit code 0 if nothing got worse
python3 $B --freecad build/bin/FreeCAD --label after --baseline baseline.json -o after.json

# compare two existing result files, or show the summary of one
python3 $B --compare baseline.json after.json
python3 $B --summary after.json

# the job of the demo part, at a medium accuracy
python3 $B --freecad build/bin/FreeCAD --jobs manifold_block --quality 5

# also measure the accuracy of the stock against the exact material (needs the tests built)
python3 $B --freecad build/bin/FreeCAD --jobs manifold_block --accuracy
```

`--quality` (1 low to 10 high, default 10) is the Accuracy slider of the simulator task panel.
`--color-mode` is its Colors: `operation` (default), `plain`, `thin-walls` or `distance` (from the
model). Thin walls and the distance are measured while they are shown, which takes time. Only
compare runs made with the same quality and colors, `--compare` warns about a difference.

Result files are machine specific and must not be committed; `*.json` in this directory is
ignored by git. `run_benchmark.py --help` lists all options.

### Verdict and exit code

The last line of the output is `VERDICT {...}`, a JSON object with:

- `status`: `pass` or `fail`, also given by the exit code (0 or 1)
- `failures`: errors, scenarios that timed out, failed correctness checks, failed accuracy runs
- `regressed`, `improved`: metrics that changed by more than `--fail-above` (default 1.25×) and
  more than `--noise-ms` (default 2 ms) compared with `--baseline`, with their ratio new/base;
  accuracy values that changed by more than `--fail-above` and more than 1 µm (p99), 2 µm (p999),
  20 µm (max), 0.5 mm³ (gross) or 2 (missed), with their new value

A run fails on any failure or regression. Improvements are reported but never fail a run.

### Quick and full mode

Quick mode (default) runs the `mixed` (all tool shapes) and `five_axis` jobs at size `small` (2k
moves each) in one FreeCAD process. A responsive simulator finishes in about a minute.

`--full` runs `pocket`, `surface3d`, `adaptive`, `tool_shapes`, `mixed` and `five_axis` at sizes `small`,
`medium` and `large`, every job in its own FreeCAD process, with more samples per scenario and 3
repeats. Repeats are combined by the median of their medians. Use `--repeats` in quick mode when a
result looks noisy.

### Flashes

`--scenarios flashes` (not run by default) plays the job while grabbing the view, and fails the run
on flashes: pixels of the stock that show the background for one grab, like cracks in the mesh
that appear while it is being updated. With `--images`, the first grabs with flashes are saved.

### Fixed workload

Every scenario does a fixed amount of work, so all runs measure the same thing:

| | quick | full |
| --- | --- | --- |
| idle frames | 20 | 30 |
| playback duration of the whole job | 10 s | 30 s |
| scrub positions per scenario | 20 | 40 |
| rotation frames | 30 | 60 |
| zoom steps in and out | 3 | 5 |
| pan steps | 12 | 20 |

Playback is the exception: it plays the whole job from start to end, at the speed that takes the
listed duration. The number of frames therefore depends on how fast the simulator is, and every
frame advances more steps than at the highest speed of the GUI (×50); the results include the
steps per frame.

`--timeout` (default 120 s) only protects against hanging: a scenario that reaches it stops, is
marked with `*` in the summary and fails the run. A slow simulator, like the original one on large
jobs, takes correspondingly longer.

### Environment

FreeCAD runs with a temporary user configuration (`-u`), so the results don't depend on personal
settings and the benchmark never changes them. The navigation style is set to OpenInventor, where
dragging with the middle mouse button pans.

Headless runs force Qt onto Xvfb (`QT_QPA_PLATFORM=xcb`, `WAYLAND_DISPLAY` removed), so no window
appears on the desktop. They render with Mesa's software renderer (llvmpipe) unless the X server
has GPU access, so absolute numbers are much slower than on a GPU. Only compare runs made on the
same machine with the same options. `--display` runs on the current display and GPU, but window
sizes may then differ from the reference images.

## Jobs

Sizes are target motion command counts: `small` 2k, `medium` 20k, `large` 100k, `huge` 400k. A
number can be given instead of a size name. Document jobs have one size, `document`, whatever
`--sizes` says.

| job | content |
| --- | --- |
| `mixed` | a complex part, every operation in its own quarter of the stock: pocket, adaptive clearing, 3D finishing and the `tool_shapes` operations |
| `five_axis` | 5 axis machining, every operation in its own quarter of the stock: a dome roughed with waterline rings and finished with a ball end mill whose axis follows the surface normal with a lead angle (simultaneous 5 axis, tilt limited to 20°), flank (swarf) milling of an island with drafted walls where the tool axis turns at the corners, grooves where the tool swivels while moving, and holes and a slot machined into the side faces with a horizontal tool |
| `pocket` | pocket with zigzag clearing and a wall finishing pass at every step down level |
| `surface3d` | ball end semi finishing and finishing of a free form surface, many short 3D moves |
| `adaptive` | trochoidal slots side by side at several depths, lots of small arcs |
| `drilling` | grid of `G81` drill cycles |
| `tool_shapes` | a pocket machined with every tool shape FreeCAD supports: roughing, bull nose floor finishing, ball end channels, a dovetail undercut and a slitting saw groove in the walls, corner radius rounding of the top edges, a bored and thread milled hole, a V groove and drilled holes |
| `manifold_block` | document job: the CAM job of `../../DemoParts/manifold_block_3plus2.fcstd` (made by `make_manifold_block_3plus2.py` there), a manifold block machined done-in-one with every tool shape and every 3 axis operation, and 3+2 work planes for the front, the right side and a 45 degree chamfer. It is given to the simulator like the simulator task panel does (`SimulationSteps` in `Path/Main/Gui/SimulatorGL.py`): the active operations in world coordinates with the tool axis of their work plane, the tool backing out and turning around the stock between work planes, the job's stock and model. Not in quick or full mode, run it with `--jobs manifold_block` |

Operations are laid out like on a real part: every operation cuts material, in its own area of the
stock or on features made by an earlier operation. 5 axis operations turn the tool only at safe
positions, the tool axis follows the convention of the simulator (`MotionToolAxis`, see
`../AppGL/README.md`). Step sizes stay realistic, at least 0.5 step
down and 1.0 stepover for pockets and 0.5 advance per loop for adaptive clearing. Like on real
jobs, most of the moves of large `mixed` jobs come from 3D finishing. The standalone `pocket` and
`adaptive` jobs therefore stop growing at about 5k and 50k moves, and `tool_shapes` and `drilling`
are smaller than the size target.

`python3 jobs.py analyze JOB SIZE` simulates a job, including tilted tools, on a coarse voxel model
(0.5 x 0.5 x 0.2) and
prints, for every operation, the fraction of feed moves that remove material and the removed
volume. Positioning feeds, like plunging into an already cleared pocket, and the return half of
trochoidal loops remove nothing by design.

## Scenarios

The simulator is driven through its own GUI controls and view navigation, so the measured code
paths are the ones a user triggers.

| scenario | what it measures |
| --- | --- |
| `load` | time from `BeginSimulation` until the first frame is shown (`load s`), and until the view shows the end of the job, with the slider moved there right after the first frame (`complete_s`, `end s`): the time a user waits for the whole job |
| `jump` | right after loading, releasing the slider at far positions (end, middle, start, ...): the time until the view shows each, before and while the background simulation makes its snapshots. What a user waits for after moving the slider; the scrub scenarios move it in small steps and run later |
| `idle` | frames without any change |
| `playback` | playing the whole job from start to end |
| `scrub_forward`, `scrub_backward`, `scrub_random` | dragging the stage slider forward, backward and to random positions, then releasing it |
| `rotate_mid`, `rotate_end` | continuous view rotation with half or all of the job simulated (`rotate_mid` in full mode only) |
| `zoom_pan` | zooming in and out, and panning with the mouse |
| `toggles` | toggling path display and ambient occlusion, showing the model, every color mode |
| `correctness` | image checks, see below |

### Measurements

- `frame_ms`: time of the simulator frame, measured inside the simulator widget
  (`CAMSim.SetFrameTiming`), including waiting for the GPU to finish.
- `latency_ms`: time from the user action until the frame is on screen (`frameSwapped`). This
  includes Qt composing the window.
- `converge_ms`, `converge_frames`: time and frames until the view shows the requested state. A
  simulator that spreads work over several frames reports this through `CAMSim.IsRenderPending`.
- `release`: for scrub scenarios, the frame and convergence after releasing the slider.
- `memory`: resident and peak resident memory of the FreeCAD process after every scenario (Linux).

Compared between runs are the time until the view shows the end of the job (`load.complete`), the
median frame time of every scenario, for the scrub scenarios the median convergence time and the
release time, and for `jump` the first, median and longest time until the view shows a jump.

While the simulator reports pending work, the benchmark waits for the frames the simulator view
asks for (its timer renders when `NeedsFrame`), like in the GUI, instead of rendering continuously:
a simulation working in other threads doesn't need frames until it has something new to show, and
continuous frames would take the CPU from it on the software renderer.

### Correctness

At 50% and 100% of the job, the view is reached by dragging the slider from the start, then the
slider is released. The check compares the image shown while dragging with the image after the
release, and the image after the release with the reference image in `reference/`. It reports the
fraction of pixels that differ by more than 40 (of 255) in any color channel, and passes when that
is at most `--max-image-diff` (default 1%). A missing reference image is reported in the results
but does not fail the run. The correctness checks always run completely, the timeout does not
apply.

The reference images depend on the window size, view, quality, colors and job. Their names have
the options that are not the defaults: `_qualityN` for another quality than 10, `_nocolors_nothin`
for plain colors, `_thinwalls` and `_distance` for the other colors, like
`mixed_small_nocolors_nothin_50.png`.

The plain `mixed` references (`_nocolors_nothin`) were rendered by the original stencil based
simulator at rest, in quick mode on Xvfb, an independent check of the material simulation: run
with `--color-mode plain` to check against them. All
other references were rendered by the material simulation itself, they only detect changes: the
original simulator can't simulate 5 axis toolpaths or show the display options. To replace them after an
intended visual change, run with `--update-reference` and commit the new images.

### Accuracy

The images only show that the simulator did not change. `--accuracy` measures how close the stock it
shows is to the material the toolpath really leaves:

1. The simulator writes every job to a file when it starts simulating it (`CAM_SIM_DUMP`,
   `Material::SaveJob`): the tool profiles, the toolpath as the simulator reads it and the stock
   mesh. `--jobs-dir` keeps these files, they are in a temporary directory otherwise.
2. After the GUI runs, `CAM_simulator_accuracy` (built with the tests, in `build/tests`) simulates
   every job again outside of the GUI, at the same quality, and measures its stock mesh against
   an oracle of the exact material (`tests/src/Mod/CAM/App/SimulatorAccuracy.h`).

The oracle decides for any point whether it holds material: inside the stock mesh and outside every
tool swept along the path. It shares no code with the material simulation except the spheres the
material simulation fits to the arcs of profiles (as FreeCAD gives a ball end as a few edges);
motions are divided into pieces within 0.1 µm of the exact sweep, turning motions too.

The accuracy table of the summary has:

- surface error: 20000 points sampled on the mesh, the distance along the mesh normal to the true
  surface, as percentiles and the largest, in µm, and the bias (mean signed error, + where the mesh
  is inside the material: it shows too much removed). The true surface is searched along the mesh
  normal, and along the axes and their diagonals where it is not found that way (a sample on a
  wall at the height of the floor it meets would search along the floor). `missed`: samples
  without the true surface within 0.5 mm, like a piece of surface where there is none.
- volume error: 1 million points in the stock box, classified by the oracle and by the mesh.
  `wrong` is the volume shown wrongly, `gross` the part of it farther than 0.05 mm from the mesh,
  like missing or misplaced surface. Small cells and thin features account for most of `wrong`.
- `triangles`: of the stock mesh.

`CAM_simulator_accuracy` also runs on its own, on a job file written by the simulator (set
`CAM_SIM_DUMP` to a file name before starting FreeCAD and open the simulator):

```sh
build/tests/CAM_simulator_accuracy manifold_block.job --quality 10
build/tests/CAM_simulator_accuracy manifold_block.job --step 50000 --gross gross.txt
```

`--step N` measures the stock at a step of the timeline, `--max-p99`, `--max-p999`, `--max-gross`
and `--max-missed` make it fail (exit code 1) beyond limits, `--json` writes the results and
`--gross` the points of gross errors, to find them in the view. `--help` lists all options.

The unit tests `MaterialAccuracy.*` (`CAM_tests_run`) run the same measurement on small jobs with
all tool shapes, 3 and 5 axis, with fixed limits, and check the oracle against cuts known in
closed form.

## Results

The result JSON contains the configuration, git commit, CPU, OpenGL renderer, view size and per
scenario statistics (`count`, `mean`, `median`, `p95`, `max`, `over_33ms` in milliseconds). The
summary shows frame times as `median/max` in milliseconds. A frame time above 33 ms (below 30 fps)
is noticeable as lag.
