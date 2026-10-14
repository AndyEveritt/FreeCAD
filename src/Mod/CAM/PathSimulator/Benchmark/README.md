# CAM simulator benchmark

Measures how responsive the CAM simulator (`CAMSimulator`, the OpenGL simulator) is on synthetic
jobs of increasing complexity, and checks that it still shows the right result. It is meant to be
run often during development, by developers and AI agents, and in CI. It is not installed with
FreeCAD.

## Files

- `jobs.py`: generates G-code jobs, tool profiles and stock. Uses only the Python standard library.
- `sim_benchmark.py`: runs inside the FreeCAD GUI, drives the simulator and records results.
- `run_benchmark.py`: launcher. Starts FreeCAD, prints a summary, compares with a baseline and
  reports a verdict.
- `reference/`: correctness reference images for the default options: `mixed` at sizes `small`
  (quick mode) and `medium`, and `five_axis` at size `small` (quick mode).

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
```

Result files are machine specific and must not be committed; `*.json` in this directory is
ignored by git. `run_benchmark.py --help` lists all options.

### Verdict and exit code

The last line of the output is `VERDICT {...}`, a JSON object with:

- `status`: `pass` or `fail`, also given by the exit code (0 or 1)
- `failures`: errors, scenarios that timed out, failed correctness checks
- `regressed`, `improved`: metrics that changed by more than `--fail-above` (default 1.25×) and
  more than `--noise-ms` (default 2 ms) compared with `--baseline`, with their ratio new/base

A run fails on any failure or regression. Improvements are reported but never fail a run.

### Quick and full mode

Quick mode (default) runs the `mixed` (all tool shapes) and `five_axis` jobs at size `small` (2k
moves each) in one FreeCAD process. A responsive simulator finishes in about a minute.

`--full` runs `pocket`, `surface3d`, `adaptive`, `tool_shapes`, `mixed` and `five_axis` at sizes `small`,
`medium` and `large`, every job in its own FreeCAD process, with more samples per scenario and 3
repeats. Repeats are combined by the median of their medians. Use `--repeats` in quick mode when a
result looks noisy.

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
number can be given instead of a size name.

| job | content |
| --- | --- |
| `mixed` | a complex part, every operation in its own quarter of the stock: pocket, adaptive clearing, 3D finishing and the `tool_shapes` operations |
| `five_axis` | 5 axis machining, every operation in its own quarter of the stock: a dome roughed with waterline rings and finished with a ball end mill whose axis follows the surface normal with a lead angle (simultaneous 5 axis, tilt limited to 20°), flank (swarf) milling of an island with drafted walls where the tool axis turns at the corners, grooves where the tool swivels while moving, and holes and a slot machined into the side faces with a horizontal tool |
| `pocket` | pocket with zigzag clearing and a wall finishing pass at every step down level |
| `surface3d` | ball end semi finishing and finishing of a free form surface, many short 3D moves |
| `adaptive` | trochoidal slots side by side at several depths, lots of small arcs |
| `drilling` | grid of `G81` drill cycles |
| `tool_shapes` | a pocket machined with every tool shape FreeCAD supports: roughing, bull nose floor finishing, ball end channels, a dovetail undercut and a slitting saw groove in the walls, corner radius rounding of the top edges, a bored and thread milled hole, a V groove and drilled holes |

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
| `load` | time from `BeginSimulation` until the first frame is shown |
| `idle` | frames without any change |
| `playback` | playing the whole job from start to end |
| `scrub_forward`, `scrub_backward`, `scrub_random` | dragging the stage slider forward, backward and to random positions, then releasing it |
| `rotate_mid`, `rotate_end` | continuous view rotation with half or all of the job simulated (`rotate_mid` in full mode only) |
| `zoom_pan` | zooming in and out, and panning with the mouse |
| `toggles` | toggling path display, ambient occlusion and stock/model display |
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

Compared between runs are the median frame time of every scenario, and for the scrub scenarios the
median convergence time and the release time.

### Correctness

At 50% and 100% of the job, the view is reached by dragging the slider from the start, then the
slider is released. The check compares the image shown while dragging with the image after the
release, and the image after the release with the reference image in `reference/`. It reports the
fraction of pixels that differ by more than 40 (of 255) in any color channel, and passes when that
is at most `--max-image-diff` (default 1%). A missing reference image is reported in the results
but does not fail the run. The correctness checks always run completely, the timeout does not
apply.

The `mixed` reference images were rendered by the original stencil based simulator at rest, in quick
mode on Xvfb, an independent check of the dexel simulation. The original simulator can't simulate
5 axis toolpaths, so the `five_axis` reference images were rendered by the dexel simulation itself,
they only detect changes. They depend on the window size, view, quality and job, so they only apply to runs with the
default options. To replace them after an intended visual change, run with `--update-reference`
and commit the new images.

## Results

The result JSON contains the configuration, git commit, CPU, OpenGL renderer, view size and per
scenario statistics (`count`, `mean`, `median`, `p95`, `max`, `over_33ms` in milliseconds). The
summary shows frame times as `median/max` in milliseconds. A frame time above 33 ms (below 30 fps)
is noticeable as lag.
