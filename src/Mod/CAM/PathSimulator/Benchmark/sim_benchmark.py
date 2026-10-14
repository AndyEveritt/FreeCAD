# SPDX-License-Identifier: LGPL-2.1-or-later

"""CAM simulator benchmark, executed inside the FreeCAD GUI by run_benchmark.py.

Reads its configuration from the JSON file in the environment variable CAMSIM_BENCH_CONFIG, runs
the requested scenarios on each configured job and writes the results to CAMSIM_BENCH_OUTPUT.
FreeCAD is terminated when done.

The simulator is driven through its GUI controls (play button, stage slider, rotate button, view
navigation), so the measured code paths are the ones a user triggers.

Every scenario does a fixed amount of work (CONFIG["counts"]), so runs on different machines and
of different simulator versions measure the same thing. A per scenario timeout is only a safety
net against hanging: a scenario that hits it stops taking samples and is marked as timed out.
"""

import json
import math
import os
import random
import sys
import time
import traceback

CONFIG = json.load(open(os.environ["CAMSIM_BENCH_CONFIG"]))
OUTPUT = os.environ["CAMSIM_BENCH_OUTPUT"]

sys.path.insert(0, os.path.dirname(os.path.abspath(CONFIG["jobs_module"])))

import FreeCAD  # noqa: E402
import FreeCADGui  # noqa: E402
import Part  # noqa: E402
import Path  # noqa: E402
import CAMSimulator  # noqa: E402
from PySide import QtCore, QtGui  # noqa: E402

import jobs  # noqa: E402

try:
    import numpy
except ImportError:
    numpy = None

app = QtGui.QApplication.instance()
results = []


def log(*args):
    print("[camsim-bench]", *args, flush=True)


def pump(seconds=0.0):
    """Process events for the given time, at least once."""
    end = time.perf_counter() + seconds
    while True:
        app.processEvents(QtCore.QEventLoop.AllEvents, 10)
        if time.perf_counter() >= end:
            break


def stats(values):
    if not values:
        return {"count": 0}
    v = sorted(values)
    n = len(v)

    def pct(p):
        return v[min(n - 1, int(round(p / 100.0 * (n - 1))))]

    return {
        "count": n,
        "mean": sum(v) / n,
        "median": pct(50),
        "p95": pct(95),
        "max": v[-1],
        "over_33ms": sum(1 for x in v if x > 33.3),
    }


def memory_mb():
    """Resident and peak resident memory of this process, Linux only."""
    info = {}
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith(("VmRSS:", "VmHWM:")):
                    key, value = line.split(":", 1)
                    name = "rss_mb" if key == "VmRSS" else "peak_rss_mb"
                    info[name] = int(value.split()[0]) / 1024
    except OSError:
        pass
    return info


def image_array(image):
    image = image.convertToFormat(QtGui.QImage.Format_RGBA8888)
    buf = numpy.frombuffer(image.constBits(), numpy.uint8, image.sizeInBytes())
    rows = buf.reshape(image.height(), image.bytesPerLine())[:, : image.width() * 4]
    return rows.reshape(image.height(), image.width(), 4)[..., :3].astype(numpy.int16)


def image_diff(a, b, threshold=40):
    """Fraction of pixels that differ noticeably, and mean absolute difference."""
    if numpy is None:
        return {"error": "numpy not available"}
    if a.size() != b.size():
        return {"error": f"size mismatch {a.width()}x{a.height()} vs {b.width()}x{b.height()}"}
    diff = numpy.abs(image_array(a) - image_array(b)).max(axis=2)
    return {"fraction": float((diff > threshold).mean()), "mean_abs": float(diff.mean())}


class Limit:
    """Safety timeout of one scenario."""

    def __init__(self, seconds):
        self.seconds = seconds
        self.start = time.perf_counter()
        self.setup_s = 0.0
        self.timed_out = False

    def elapsed(self):
        return time.perf_counter() - self.start

    def restart(self):
        """Start the timeout now, so the setup of a scenario does not count."""
        self.setup_s = self.elapsed()
        self.start = time.perf_counter()

    def expired(self):
        if self.elapsed() > self.seconds:
            self.timed_out = True
        return self.timed_out


class JobBench:
    def __init__(self, job, size):
        self.job = job
        self.size = size
        self.result = {"job": job, "size": size, "scenarios": {}, "errors": []}
        self.sim = CAMSimulator.PathSim()
        self.glw = None
        self.buttons = {}
        self.slider = None
        self.swaps = []
        self.has_pending = hasattr(self.sim, "IsRenderPending")
        self.counts = CONFIG["counts"]

    # setup

    def load(self):
        job = jobs.make_job(self.job, self.size)
        doc = FreeCAD.newDocument("CamSimBenchmark")
        pump(0.2)

        t0 = time.perf_counter()
        commands = [Path.Command(line) for line in job["gcode"]]
        self.result["gcode_lines"] = len(commands)
        self.result["command_build_s"] = time.perf_counter() - t0

        x, y, z, length, width, height = job["stock"]
        stock = Part.makeBox(length, width, height, FreeCAD.Vector(x, y, z))

        t0 = time.perf_counter()
        self.sim.ResetSimulation(FreeCADGui.getDocument(doc))
        for number, diameter, profile in job["tools"]:
            self.sim.AddTool(profile, number, diameter, 1)
        for cmd in commands:
            self.sim.AddCommand(cmd)
        self.result["add_commands_s"] = time.perf_counter() - t0

        pump(0.2)
        self.find_widgets()
        self.sim.SetFrameTiming(True)
        memory_before = memory_mb()

        # time from starting the simulation until the first frame shows it
        self.sim.TakeFrameTimes()
        t0 = time.perf_counter()
        self.sim.BeginSimulation(stock, CONFIG["quality"])
        sample = self.wait_frame()
        self.result["scenarios"]["load"] = {
            "wall_s": time.perf_counter() - t0,
            "first_frame_ms": sample["frame_ms"],
            "memory_before": memory_before,
            "memory": memory_mb(),
        }
        self.settle()

        self.result["renderer"] = self.sim.GetRendererInfo()
        self.result["total_steps"] = self.slider.maximum()
        log("loaded", self.job, self.size, self.result["gcode_lines"], "lines")

    def find_widgets(self):
        widgets = app.allWidgets()
        gl = [w for w in widgets if w.metaObject().className() == "CAMSimulator::DlgCAMSimulator"]
        if not gl:
            raise RuntimeError("CAM simulator widget not found")
        self.glw = gl[0]
        self.buttons = {w.objectName(): w for w in widgets if isinstance(w, QtGui.QToolButton)}
        self.slider = [
            w for w in widgets if isinstance(w, QtGui.QSlider) and w.objectName() == "stageSlider"
        ][0]

        try:
            QtCore.QObject.connect(
                self.glw,
                QtCore.SIGNAL("frameSwapped()"),
                lambda: self.swaps.append(time.perf_counter()),
            )
        except Exception:  # latency is optional
            self.swaps = None

    @staticmethod
    def viewer():
        """The widget receiving the mouse events of the simulator view navigation.

        Looked up every time, the viewer may recreate it.
        """

        def in_simulator(w):
            while w is not None:
                if w.metaObject().className() == "Gui::MDIViewWithCamera":
                    return True
                w = w.parentWidget()
            return False

        # the viewport of the simulator's navigation viewer, taken from the live widgets because
        # wrappers returned by viewport() can refer to an already deleted widget
        for w in app.allWidgets():
            parent = w.parentWidget()
            if (
                parent is not None
                and parent.metaObject().className() == "Gui::View3DInventorViewer"
                and in_simulator(parent)
                and w.isVisible()
                and w.width() > 1
            ):
                return w
        return None

    # frame helpers

    def wait_frame(self, timeout=None):
        """Wait for the next simulator frame, and until the view shows the current state.

        Callers discard recorded frames (TakeFrameTimes) before changing the simulator state, so
        the first new frame is the one reacting to the change. Returns frame_ms of that frame,
        latency_ms until it was on screen, and converge_ms/converge_frames until the simulator no
        longer reported pending work.
        """
        timeout = timeout or CONFIG["frame_timeout"]
        t0 = time.perf_counter()
        end = t0 + timeout
        if self.swaps is not None:
            self.swaps.clear()
        self.glw.update()
        frames = []
        while True:
            if time.perf_counter() > end:
                raise RuntimeError("simulator did not finish a frame within %.0f s" % timeout)
            app.processEvents(QtCore.QEventLoop.AllEvents, 5)
            new = self.sim.TakeFrameTimes()
            if new:
                frames += new
                if not (self.has_pending and self.sim.IsRenderPending()):
                    break
                self.glw.update()

        sample = {
            "frame_ms": frames[0],
            "converge_ms": (time.perf_counter() - t0) * 1000,
            "converge_frames": len(frames),
        }
        if self.swaps:
            sample["latency_ms"] = (self.swaps[0] - t0) * 1000
        return sample

    def settle(self):
        """Render until the simulator shows the current state, then drop recorded frames."""
        self.sim.TakeFrameTimes()
        self.wait_frame()
        pump(0.05)
        self.sim.TakeFrameTimes()

    def action_samples(self, actions, limit):
        """Run actions, each followed by waiting for the frame showing its result."""
        samples = []
        for action in actions:
            if limit.expired():
                break
            t0 = time.perf_counter()
            self.sim.TakeFrameTimes()
            action()
            sample = self.wait_frame()
            sample["wall_ms"] = (time.perf_counter() - t0) * 1000
            samples.append(sample)
        return samples

    def continuous_frames(self, count, limit, done=lambda: False):
        """Collect the times of frames rendered by the simulator itself, e.g. while playing."""
        times = []
        while len(times) < count and not limit.expired() and not done():
            pump(0.01)
            times += self.sim.TakeFrameTimes()
        return times[:count]

    @staticmethod
    def summarize(samples):
        out = {"frame_ms": stats([s["frame_ms"] for s in samples])}
        for key in ("latency_ms", "converge_ms", "converge_frames", "wall_ms"):
            values = [s[key] for s in samples if key in s]
            if values:
                out[key] = stats(values)
        return out

    # GUI actions

    def toggle_play(self):
        self.buttons["playButton"].click()

    def set_speed(self, speed):
        """Set the playback speed, like the speed buttons of the simulator do."""
        gui = [
            w for w in app.allWidgets() if w.metaObject().className() == "CAMSimulator::GuiDisplay"
        ][0]
        QtCore.QMetaObject.invokeMethod(
            gui, "speedChanged", QtCore.Qt.DirectConnection, QtCore.Q_ARG(int, speed)
        )

    def move_to(self, value):
        """Move the stage like a user clicking and releasing the slider."""
        self.slider.setSliderDown(True)
        self.slider.setSliderPosition(value)
        self.slider.setSliderDown(False)
        self.settle()

    def set_view(self, view):
        FreeCADGui.SendMsgToActiveView(view)
        pump(1.0)  # let the camera animation finish
        FreeCADGui.SendMsgToActiveView("ViewFit")
        pump(0.5)
        self.settle()

    def mouse(self, kind, pos, button, buttons):
        viewer = self.viewer()
        event = QtGui.QMouseEvent(
            kind, pos, viewer.mapToGlobal(pos), button, buttons, QtCore.Qt.NoModifier
        )
        app.sendEvent(viewer, event)

    # scenarios

    def scenario_idle(self, limit):
        self.settle()
        return self.summarize(self.action_samples([lambda: None] * self.counts["idle"], limit))

    def scenario_playback(self, limit):
        """Play the whole job, at a speed that takes about CONFIG["playback_seconds"]."""
        total = self.slider.maximum()
        self.move_to(0)
        # the simulator advances speed * 60 steps per second
        speed = max(1, math.ceil(total / (CONFIG["playback_seconds"] * 60)))
        self.set_speed(speed)
        limit.restart()
        self.sim.TakeFrameTimes()
        t0 = time.perf_counter()
        self.toggle_play()
        times = []
        while self.slider.value() < total and not limit.expired():
            pump(0.01)
            times += self.sim.TakeFrameTimes()
        elapsed = time.perf_counter() - t0
        reached_end = self.slider.value() >= total
        # playing stops by itself at the end of the simulation
        if not reached_end:
            self.toggle_play()
        times += self.sim.TakeFrameTimes()
        result = {
            "frame_ms": stats(times),
            "fps": len(times) / elapsed,
            "speed": speed,
            "steps_per_frame": self.slider.value() / max(1, len(times)),
            "reached_end": reached_end,
        }
        self.set_speed(1)
        self.settle()
        return result

    def scenario_flashes(self, limit):
        """Play the job and look for flashes: parts of the stock that show the background for a
        frame. The view is grabbed while playing, a flash pixel has the background color and
        shows the stock in the grabs before and after. The stock colors are saturated, unlike the
        tool and the background."""
        if numpy is None:
            return {"error": "numpy not available"}
        total = self.slider.maximum()
        self.move_to(0)
        speed = max(1, math.ceil(total / (CONFIG["playback_seconds"] * 60)))
        self.set_speed(speed)
        limit.restart()
        self.toggle_play()
        grabs = []
        background = None
        flashes = []
        count = 0
        while self.slider.value() < total and not limit.expired():
            pump(0.005)
            image = self.glw.grab().toImage()
            pixels = image_array(image)
            if background is None:
                background = pixels[2, 2].copy()
            grabs = (grabs + [(image, pixels)])[-3:]
            count += 1
            if len(grabs) < 3:
                continue
            a, b, c = (g[1] for g in grabs)
            changed = (numpy.abs(b - a).max(axis=2) > 60) & (numpy.abs(b - c).max(axis=2) > 60)
            stable = numpy.abs(a - c).max(axis=2) < 20

            def saturated(p):
                return p.max(axis=2) - p.min(axis=2) > 30

            stable &= saturated(a) & saturated(c)
            empty = numpy.abs(b - background).max(axis=2) < 20
            pixels_flashing = int((changed & stable & empty).sum())
            if pixels_flashing > 20:
                flashes.append({"grab": count, "pixels": pixels_flashing, "step": self.slider.value()})
                if CONFIG.get("images_dir") and len(flashes) <= 3:
                    for k, (img, _) in enumerate(grabs):
                        img.save(os.path.join(CONFIG["images_dir"], f"{self.job}_{self.size}_flash{len(flashes)}_{k}.png"))
        if self.slider.value() < total:
            self.toggle_play()
        self.set_speed(1)
        self.settle()
        return {"grabs": count, "flashes": len(flashes), "worst": max((f["pixels"] for f in flashes), default=0), "details": flashes[:10]}

    def scrub(self, positions, limit):
        """Drag the slider through the positions, waiting for the view at each."""
        limit.restart()
        self.slider.setSliderDown(True)
        actions = [lambda v=v: self.slider.setSliderPosition(v) for v in positions]
        samples = self.action_samples(actions, limit)
        self.sim.TakeFrameTimes()
        t0 = time.perf_counter()
        self.slider.setSliderDown(False)
        release = self.wait_frame()
        release["wall_ms"] = (time.perf_counter() - t0) * 1000
        result = self.summarize(samples)
        result["release"] = release
        result["positions"] = len(samples)
        return result

    def scrub_positions(self, fractions):
        total = self.slider.maximum()
        return [round(total * f) for f in fractions]

    def scenario_scrub_forward(self, limit):
        n = self.counts["scrub"]
        self.move_to(0)
        return self.scrub(self.scrub_positions([(i + 1) / n for i in range(n)]), limit)

    def scenario_scrub_backward(self, limit):
        n = self.counts["scrub"]
        self.move_to(self.slider.maximum())
        return self.scrub(self.scrub_positions([(n - 1 - i) / n for i in range(n)]), limit)

    def scenario_scrub_random(self, limit):
        rng = random.Random(1234)
        positions = self.scrub_positions([rng.random() for _ in range(self.counts["scrub"])])
        return self.scrub(positions, limit)

    def rotate(self, stage, limit):
        self.move_to(round(self.slider.maximum() * stage))
        limit.restart()
        self.sim.TakeFrameTimes()
        rotate = self.buttons["rotateButton"]
        t0 = time.perf_counter()
        rotate.setChecked(True)
        times = self.continuous_frames(self.counts["rotate"], limit)
        rotate.setChecked(False)
        elapsed = time.perf_counter() - t0
        self.set_view(CONFIG["view"])
        return {"frame_ms": stats(times), "fps": len(times) / elapsed}

    def scenario_rotate_mid(self, limit):
        return self.rotate(0.5, limit)

    def scenario_rotate_end(self, limit):
        return self.rotate(1.0, limit)

    def scenario_zoom_pan(self, limit):
        self.move_to(self.slider.maximum())
        limit.restart()
        actions = []
        for _ in range(self.counts["zoom"]):
            actions.append(lambda: FreeCADGui.SendMsgToActiveView("ZoomIn"))
        for _ in range(self.counts["zoom"]):
            actions.append(lambda: FreeCADGui.SendMsgToActiveView("ZoomOut"))
        viewer = self.viewer()
        if viewer is not None:
            center = QtCore.QPointF(viewer.width() / 2, viewer.height() / 2)
            actions.append(
                lambda: self.mouse(
                    QtCore.QEvent.MouseButtonPress,
                    center,
                    QtCore.Qt.MiddleButton,
                    QtCore.Qt.MiddleButton,
                )
            )
            for i in range(1, self.counts["pan"] + 1):
                pos = center + QtCore.QPointF(12 * i, 6 * i)
                actions.append(
                    lambda pos=pos: self.mouse(
                        QtCore.QEvent.MouseMove, pos, QtCore.Qt.NoButton, QtCore.Qt.MiddleButton
                    )
                )
        samples = self.action_samples(actions, limit)
        if viewer is not None:
            self.mouse(
                QtCore.QEvent.MouseButtonRelease,
                QtCore.QPointF(0, 0),
                QtCore.Qt.MiddleButton,
                QtCore.Qt.NoButton,
            )
        self.set_view(CONFIG["view"])
        return self.summarize(samples)

    def scenario_toggles(self, limit):
        self.move_to(self.slider.maximum())
        limit.restart()
        actions = []
        for name in ("pathButton", "ssaoButton", "stockModelButton"):
            if name in self.buttons:
                # each toggle twice, to restore the original state
                actions += [self.buttons[name].click, self.buttons[name].click]
        return self.summarize(self.action_samples(actions, limit))

    def scenario_correctness(self, limit):
        """Compare interactive frames with resting frames and with reference images.

        Always runs all checks, the timeout does not apply.
        """
        out = {"checks": []}
        for stage in CONFIG["correctness_stages"]:
            value = round(self.slider.maximum() * stage)
            # reach the position while dragging the slider from the start
            self.move_to(0)
            self.slider.setSliderDown(True)
            self.sim.TakeFrameTimes()
            self.slider.setSliderPosition(value)
            self.wait_frame()
            interactive = self.glw.grab().toImage()
            self.sim.TakeFrameTimes()
            self.slider.setSliderDown(False)
            self.wait_frame()
            resting = self.glw.grab().toImage()

            check = {"stage": stage, "interactive_vs_resting": image_diff(interactive, resting)}
            name = f"{self.job}_{self.size}_{int(stage * 100)}.png"
            if CONFIG.get("save_reference_dir"):
                resting.save(os.path.join(CONFIG["save_reference_dir"], name))
            elif CONFIG.get("reference_dir"):
                path = os.path.join(CONFIG["reference_dir"], name)
                if os.path.exists(path):
                    check["vs_reference"] = image_diff(resting, QtGui.QImage(path))
                else:
                    check["vs_reference"] = {"missing": name}
            out["checks"].append(check)

        diffs = [
            d
            for c in out["checks"]
            for d in (c.get("interactive_vs_resting"), c.get("vs_reference"))
            if d
        ]
        fractions = [d["fraction"] for d in diffs if "fraction" in d]
        errors = [d["error"] for d in diffs if "error" in d]
        out["max_diff_fraction"] = max(fractions) if fractions else None
        out["errors"] = errors
        out["passed"] = not errors and all(f <= CONFIG["max_image_diff"] for f in fractions)
        return out

    def screenshot(self, name):
        if CONFIG.get("images_dir"):
            path = os.path.join(CONFIG["images_dir"], f"{self.job}_{self.size}_{name}.png")
            self.glw.grab().save(path)

    def run(self):
        start = time.perf_counter()
        self.load()
        self.set_view(CONFIG["view"])
        # warm up: a few frames that are not measured
        for _ in range(3):
            self.settle()

        for name in CONFIG["scenarios"]:
            log("scenario", name)
            limit = Limit(CONFIG["timeout"])
            try:
                result = getattr(self, "scenario_" + name)(limit)
                result["duration_s"] = limit.elapsed()
                result["setup_s"] = limit.setup_s
                result["timed_out"] = limit.timed_out
                result["memory"] = memory_mb()
                self.result["scenarios"][name] = result
                self.screenshot(name)
            except Exception:
                self.result["errors"].append(f"{name}: {traceback.format_exc()}")
                log(traceback.format_exc())
                self.settle()
        self.result["duration_s"] = time.perf_counter() - start
        return self.result


def setup_window():
    mw = FreeCADGui.getMainWindow()
    width, height = CONFIG["window"]
    mw.showNormal()
    # a fixed size, so window managers and saved window states can't change it
    mw.setFixedSize(width, height)
    for widget in mw.findChildren(QtGui.QDockWidget) + mw.findChildren(QtGui.QToolBar):
        widget.hide()
    mw.statusBar().hide()
    pump(0.5)


def save():
    with open(OUTPUT, "w") as f:
        json.dump(results, f, indent=1)


def main():
    try:
        # a known navigation style, where dragging with the middle mouse button pans. The launcher
        # runs FreeCAD with a temporary user configuration, so this does not change user settings.
        FreeCAD.ParamGet("User parameter:BaseApp/Preferences/View").SetString(
            "NavigationStyle", "Gui::OpenInventorNavigationStyle"
        )
        setup_window()
        for run in CONFIG["runs"]:
            try:
                results.append(JobBench(run["job"], run["size"]).run())
            except Exception:
                results.append(
                    {"job": run["job"], "size": run["size"], "errors": [traceback.format_exc()]}
                )
                log(traceback.format_exc())
            save()
            # each job in its own document and simulator view
            for name in list(FreeCAD.listDocuments()):
                FreeCAD.closeDocument(name)
            pump(0.3)
    except Exception:
        results.append({"job": None, "size": None, "errors": [traceback.format_exc()]})
        log(traceback.format_exc())
    finally:
        save()
        os._exit(0)


# give the main window time to come up before starting
QtCore.QTimer.singleShot(500, main)
