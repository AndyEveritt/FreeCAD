#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1-or-later

"""Run the CAM simulator benchmark.

The default quick mode runs one complex job in a single FreeCAD process. --full runs every job and
size in its own FreeCAD process, with more samples and repeats. Every run checks the simulator
images against the reference images in the reference directory. See README.md for details.

With --accuracy, every job is also simulated outside of the GUI by CAM_simulator_accuracy (built
with the tests) and the stock is measured against the exact material of the job.

The last output line is a machine readable verdict, and the exit code is 0 when the run passed:
no errors, no timeouts, images match the references and, with --baseline, no scenario got slower
and no accuracy got worse.

Examples:
    run_benchmark.py --freecad build/bin/FreeCAD -o baseline.json
    run_benchmark.py --freecad build/bin/FreeCAD --baseline baseline.json -o after.json
    run_benchmark.py --compare baseline.json after.json
"""

import argparse
import datetime
import json
import os
import platform
import shutil
import statistics
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REFERENCE_DIR = os.path.join(HERE, "reference")
sys.path.insert(0, HERE)

import jobs  # noqa: E402

SCENARIOS = [
    "jump",
    "idle",
    "playback",
    "scrub_forward",
    "scrub_backward",
    "scrub_random",
    "rotate_mid",
    "rotate_end",
    "zoom_pan",
    "toggles",
    "correctness",
]
# diagnostics, run only when asked for with --scenarios
EXTRA_SCENARIOS = ["flashes"]
SCRUBS = ("scrub_forward", "scrub_backward", "scrub_random")
# the colors of the stock: the option, and its value of SimulatorColorMode
COLOR_MODES = {"plain": 0, "operation": 1, "thin-walls": 2, "distance": 3}

QUICK = {
    "jobs": "mixed,five_axis",
    "sizes": "small",
    "window": "1280x800",
    "repeats": 1,
    "scenarios": [s for s in SCENARIOS if s != "rotate_mid"],
    "counts": {"idle": 20, "scrub": 20, "rotate": 30, "zoom": 3, "pan": 12},
    "playback_seconds": 10,
}
FULL = {
    "jobs": "pocket,surface3d,adaptive,tool_shapes,mixed,five_axis",
    "sizes": "small,medium,large",
    "window": "1280x800",
    "repeats": 3,
    "scenarios": SCENARIOS,
    "counts": {"idle": 30, "scrub": 40, "rotate": 60, "zoom": 5, "pan": 20},
    "playback_seconds": 30,
}


def git_info():
    def git(*args):
        try:
            return subprocess.check_output(["git", *args], cwd=HERE, text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            return None

    return {
        "commit": git("rev-parse", "--short", "HEAD"),
        "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
    }


def cpu_name():
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def run_process(args, runs, workdir, name):
    """Run the benchmark for a list of job/size pairs in one FreeCAD process."""
    config = {
        "runs": runs,
        "quality": args.quality,
        "color_mode": args.color_mode,
        "window": args.window,
        "view": args.view,
        "scenarios": args.scenarios,
        "counts": args.counts,
        "playback_seconds": args.playback_seconds,
        "timeout": args.timeout,
        "frame_timeout": args.frame_timeout,
        "correctness_stages": [0.5, 1.0],
        "max_image_diff": args.max_image_diff,
        "jobs_module": os.path.join(HERE, "jobs.py"),
        "dump_dir": args.dump_dir,
        "images_dir": args.images,
        "reference_dir": args.reference,
        "save_reference_dir": args.save_reference,
    }
    config_path = os.path.join(workdir, f"config_{name}.json")
    output_path = os.path.join(workdir, f"result_{name}.json")
    log_path = os.path.join(workdir, f"log_{name}.txt")
    with open(config_path, "w") as f:
        json.dump(config, f)

    env = dict(os.environ)
    env["CAMSIM_BENCH_CONFIG"] = config_path
    env["CAMSIM_BENCH_OUTPUT"] = output_path

    # a temporary user configuration: reproducible settings, user settings stay untouched
    user_cfg = os.path.join(workdir, f"user_{name}.cfg")
    cmd = [args.freecad, "-u", user_cfg, os.path.join(HERE, "sim_benchmark.py")]
    if not args.display:
        # make sure Qt uses Xvfb and does not open a window on a Wayland session
        env["QT_QPA_PLATFORM"] = "xcb"
        env.pop("WAYLAND_DISPLAY", None)
        width, height = args.window
        cmd = ["xvfb-run", "-a", "-s", f"-screen 0 {width + 200}x{height + 200}x24"] + cmd

    # safety net only, every scenario has its own timeout
    timeout = (len(args.scenarios) + 2) * args.timeout * len(runs) + 300
    with open(log_path, "w") as log:
        try:
            subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
            error = None
        except subprocess.TimeoutExpired:
            error = f"timed out after {timeout:.0f} s"

    results = []
    if os.path.exists(output_path):
        with open(output_path) as f:
            results = json.load(f)
    done = {(r["job"], r["size"]) for r in results}
    for run in runs:
        if (run["job"], run["size"]) not in done:
            with open(log_path) as f:
                tail = f.read()[-2000:]
            results.append({**run, "errors": [f"{error or 'no result written'}\n{tail}"]})
    return results


# evaluation


def merge_repeats(runs):
    """Group runs by job/size. Medians of repeats are combined by their median."""
    groups = {}
    for run in runs:
        groups.setdefault((run["job"], str(run["size"])), []).append(run)
    merged = []
    for group in groups.values():
        base = json.loads(json.dumps(group[0]))
        base["repeats"] = len(group)
        base["errors"] = [e for r in group for e in r.get("errors", [])]
        for name, sc in base.get("scenarios", {}).items():
            others = [r.get("scenarios", {}).get(name, {}) for r in group]
            sc["timed_out"] = any(o.get("timed_out") for o in others)
            for key in ("frame_ms", "converge_ms"):
                medians = [o[key]["median"] for o in others if o.get(key, {}).get("count")]
                if medians and key in sc:
                    middle = statistics.median(medians)
                    sc[key]["median"] = middle
                    sc[key]["spread"] = (max(medians) - min(medians)) / max(middle, 1e-9)
            releases = [o["release"]["converge_ms"] for o in others if o.get("release")]
            if releases:
                sc["release"]["converge_ms"] = statistics.median(releases)
            if name == "correctness":
                sc["passed"] = all(o.get("passed") is not False for o in others)
            if name == "jump":
                for key in ("first_ms", "median_ms", "max_ms"):
                    values = [o[key] for o in others if o.get(key) is not None]
                    if values:
                        sc[key] = statistics.median(values)
        merged.append(base)
    return merged


def metrics(run):
    """The values compared between runs, in ms: {name: value}."""
    out = {}
    for name, sc in run.get("scenarios", {}).items():
        if name == "load" and sc.get("complete_s") is not None:
            out["load.complete"] = sc["complete_s"] * 1000
        if sc.get("frame_ms", {}).get("count"):
            out[f"{name}.frame"] = sc["frame_ms"]["median"]
        if name == "jump" and sc.get("max_ms") is not None:
            out["jump.first"] = sc["first_ms"]
            out["jump.median"] = sc["median_ms"]
            out["jump.max"] = sc["max_ms"]
        if name in SCRUBS:
            if sc.get("converge_ms", {}).get("count"):
                out[f"{name}.converge"] = sc["converge_ms"]["median"]
            if sc.get("release"):
                out[f"{name}.release"] = sc["release"]["converge_ms"]
    return out


# accuracy values compared between runs, and the differences that are never a regression
ACCURACY_NOISE = {"p99 um": 1.0, "p999 um": 2.0, "max um": 20.0, "gross mm3": 0.5, "missed": 2}


def accuracy_metrics(run):
    """The accuracy values compared between runs: {name: value}."""
    acc = run.get("accuracy")
    if not acc or "surface" not in acc:
        return {}
    surface, volume = acc["surface"], acc["volume"]
    return {
        "p99 um": surface["p99"] * 1000,
        "p999 um": surface["p999"] * 1000,
        "max um": surface["max"] * 1000,
        "gross mm3": volume["gross_mm3"],
        "missed": surface["missed"],
    }


def run_accuracy(args, data):
    """Measure the accuracy of every job the GUI wrote, once per job and size."""
    done = set()
    for run in data["runs"]:
        key = (run["job"], str(run["size"]))
        path = run.get("job_file")
        if key in done or not path:
            continue
        done.add(key)
        if not os.path.exists(path):
            run["accuracy"] = {"error": "the simulator wrote no job file"}
            continue
        print(f"accuracy of {run['job']} {run['size']} ...", flush=True)
        out = os.path.join(args.dump_dir, f"{run['job']}_{run['size']}.accuracy.json")
        cmd = [args.accuracy_tool, path, "--quality", str(args.quality), "--json", out]
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
            if proc.returncode != 0 or not os.path.exists(out):
                run["accuracy"] = {"error": (proc.stdout + proc.stderr)[-2000:]}
            else:
                with open(out) as f:
                    run["accuracy"] = json.load(f)
        except (OSError, subprocess.TimeoutExpired) as e:
            run["accuracy"] = {"error": str(e)}


def evaluate(data, base=None, fail_above=1.25, noise_ms=2.0):
    """Verdict of a run, optionally compared with a baseline run."""
    verdict = {"status": "pass", "failures": [], "regressed": {}, "improved": {}}
    runs = merge_repeats(data["runs"])
    for run in runs:
        key = f"{run['job']}:{run['size']}"
        for err in run.get("errors", []):
            verdict["failures"].append(f"{key} error: {err.strip().splitlines()[-1]}")
        for name, sc in run.get("scenarios", {}).items():
            if sc.get("timed_out"):
                verdict["failures"].append(f"{key} {name} timed out")
        flashes = run.get("scenarios", {}).get("flashes")
        if flashes and flashes.get("flashes"):
            verdict["failures"].append(
                f"{key} {flashes['flashes']} flashes, up to {flashes.get('worst')} pixels"
            )
        if run.get("accuracy", {}).get("error"):
            error = run["accuracy"]["error"].strip().splitlines() or ["failed"]
            verdict["failures"].append(f"{key} accuracy: {error[-1]}")
        corr = run.get("scenarios", {}).get("correctness")
        if corr and corr.get("passed") is False:
            verdict["failures"].append(
                f"{key} correctness failed: max diff {corr.get('max_diff_fraction')}"
                f" {'; '.join(corr.get('errors', []))}".rstrip()
            )

    if base is not None:
        base_runs = {(r["job"], str(r["size"])): r for r in merge_repeats(base["runs"])}
        for run in runs:
            b = base_runs.get((run["job"], str(run["size"])))
            if not b:
                continue
            key = f"{run['job']}:{run['size']}"
            new_metrics, base_metrics = metrics(run), metrics(b)
            for name, value in new_metrics.items():
                old = base_metrics.get(name)
                if old is None:
                    continue
                ratio = value / max(old, 1e-9)
                if ratio > fail_above and value - old > noise_ms:
                    verdict["regressed"][f"{key} {name}"] = round(ratio, 3)
                elif ratio < 1 / fail_above and old - value > noise_ms:
                    verdict["improved"][f"{key} {name}"] = round(ratio, 3)
            new_acc, base_acc = accuracy_metrics(run), accuracy_metrics(b)
            for name, value in new_acc.items():
                old = base_acc.get(name)
                if old is None:
                    continue
                noise = ACCURACY_NOISE[name]
                if value > old * fail_above and value - old > noise:
                    verdict["regressed"][f"{key} accuracy {name}"] = round(value, 3)
                elif value < old / fail_above and old - value > noise:
                    verdict["improved"][f"{key} accuracy {name}"] = round(value, 3)

    if verdict["failures"] or verdict["regressed"]:
        verdict["status"] = "fail"
    return verdict


# output


def cell(sc, key="frame_ms"):
    if sc is None:
        return "-"
    if "flashes" in sc:
        return f"{sc['flashes']} in {sc.get('grabs', 0)}"
    if "jumps_ms" in sc:
        if sc.get("max_ms") is None:
            return "-"
        text = f"{sc['first_ms']:.0f}/{sc['max_ms']:.0f}"
        return text + ("*" if sc.get("timed_out") else "")
    f = sc.get(key, {})
    if not f.get("count"):
        return "-"
    text = f"{f['median']:.0f}/{f['max']:.0f}"
    return text + ("*" if sc.get("timed_out") else "")


def display_options(data):
    """The display options of a run. Results from before they were options were plain."""
    config = data["config"]
    if "color_mode" in config:
        return "colors: " + config["color_mode"]
    # results from before the color modes
    on = [
        k
        for k, key in (("operation", "color_by_operation"), ("thin walls", "thin_walls"))
        if config.get(key)
    ]
    return "colors: " + (" and ".join(on) or "plain")


def print_summary(data, file=sys.stdout):
    git = data["git"]
    print(
        f"\nlabel: {data.get('label')}  commit: {git.get('commit')}"
        f"{' (dirty)' if git.get('dirty') else ''}  mode: {data['config']['mode']}"
        f"  quality: {data['config'].get('quality')}  {display_options(data)}"
        f"  wall time: {data.get('wall_s', 0):.0f} s",
        file=file,
    )
    renderers = {r.get("renderer", {}).get("renderer") for r in data["runs"]}
    views = {
        f"{r['renderer']['width']}x{r['renderer']['height']}"
        for r in data["runs"]
        if r.get("renderer")
    }
    print(
        f"cpu: {data['host']['cpu']}  renderer: {', '.join(sorted(x for x in renderers if x))}"
        f"  view: {', '.join(sorted(views))}\n",
        file=file,
    )

    columns = [s for s in data["config"]["scenarios"] if s != "correctness"]
    header = f"{'job':12s} {'size':7s} {'load s':>6s} {'end s':>6s}"
    header += "".join(f" {s.replace('playback', 'play'):>14s}" for s in columns)
    header += f" {'converge':>9s} {'release':>8s} {'mem MB':>7s} {'image':>7s}"
    print(header, file=file)
    for run in merge_repeats(data["runs"]):
        scenarios = run.get("scenarios", {})
        line = f"{run['job']:12s} {str(run['size']):7s}"
        line += f" {scenarios.get('load', {}).get('wall_s', 0):6.1f}"
        complete = scenarios.get("load", {}).get("complete_s")
        line += f" {complete:6.1f}" if complete is not None else f" {'-':>6s}"
        line += "".join(f" {cell(scenarios.get(s)):>14s}" for s in columns)
        scrubs = [scenarios[s] for s in SCRUBS if s in scenarios]
        converge = [s["converge_ms"]["median"] for s in scrubs if s.get("converge_ms")]
        release = [s["release"]["converge_ms"] for s in scrubs if s.get("release")]
        line += f" {max(converge):9.0f}" if converge else f" {'-':>9s}"
        line += f" {max(release):8.0f}" if release else f" {'-':>8s}"
        peaks = [s.get("memory", {}).get("peak_rss_mb", 0) for s in scenarios.values()]
        line += f" {max(peaks, default=0):7.0f}"
        corr = scenarios.get("correctness", {})
        if corr.get("max_diff_fraction") is not None:
            mark = "o" if corr.get("passed") else "f"
            line += f" {corr['max_diff_fraction'] * 100:5.2f}%{mark}"
        else:
            line += f" {'-':>7s}"
        print(line, file=file)

    accuracy = [r for r in merge_repeats(data["runs"]) if r.get("accuracy")]
    if accuracy:
        print(
            f"\n{'job':12s} {'size':7s} {'p50 um':>7s} {'p99 um':>7s} {'p999 um':>8s}"
            f" {'max um':>7s} {'bias um':>8s} {'missed':>6s} {'wrong mm3':>10s} {'gross mm3':>10s}"
            f" {'triangles':>10s}",
            file=file,
        )
        for run in accuracy:
            acc = run["accuracy"]
            line = f"{run['job']:12s} {str(run['size']):7s}"
            if "surface" not in acc:
                print(line + " error", file=file)
                continue
            s, v = acc["surface"], acc["volume"]
            line += f" {s['p50'] * 1000:7.1f} {s['p99'] * 1000:7.1f} {s['p999'] * 1000:8.1f}"
            line += f" {s['max'] * 1000:7.0f} {s['bias'] * 1000:+8.1f} {s['missed']:6d}"
            line += f" {v['misclassified_mm3']:10.2f} {v['gross_mm3']:10.2f} {acc['triangles']:10d}"
            print(line, file=file)

    print(
        "\nframe times in ms as median/max, * = timed out."
        "\njump: time until the view shows a far slider position right after loading, first/max."
        "\nconverge: median time until the view shows a scrub position (slowest scrub)."
        "\nrelease: time until the view is final after releasing the slider (slowest scrub)."
        "\nimage: largest fraction of differing pixels in the correctness checks, o=ok f=FAIL",
        file=file,
    )
    if accuracy:
        print(
            "accuracy: surface error along the normals of the stock mesh against the exact"
            "\nmaterial (percentiles, max, bias: + = too much removed), missed = samples without"
            "\nthe true surface within 0.5 mm, wrong = volume shown wrongly, gross = farther than"
            "\n0.05 mm from the mesh. See README.md.",
            file=file,
        )


def print_comparison(base, new, file=sys.stdout):
    print(
        f"\nbase: {base.get('label')} ({base['git'].get('commit')})"
        f"   new: {new.get('label')} ({new['git'].get('commit')})",
        file=file,
    )
    quality = (base["config"].get("quality"), new["config"].get("quality"))
    if quality[0] != quality[1]:
        print(f"warning: different quality, base {quality[0]}, new {quality[1]}", file=file)
    options = (display_options(base), display_options(new))
    if options[0] != options[1]:
        print(f"warning: different display, base {options[0]}, new {options[1]}", file=file)
    base_runs = {(r["job"], str(r["size"])): r for r in merge_repeats(base["runs"])}
    for run in merge_repeats(new["runs"]):
        b = base_runs.get((run["job"], str(run["size"])))
        if not b:
            continue
        print(f"\n{run['job']} {run['size']}: median ms, base -> new (ratio)", file=file)
        new_metrics, base_metrics = metrics(run), metrics(b)
        for name in sorted(set(new_metrics) | set(base_metrics)):
            old, value = base_metrics.get(name), new_metrics.get(name)
            if old is None or value is None:
                print(f"  {name:28s} {old or '-':>10} -> {value or '-'}", file=file)
            else:
                print(
                    f"  {name:28s} {old:10.1f} -> {value:10.1f}  ({value / max(old, 1e-9):.3f})",
                    file=file,
                )
        new_acc, base_acc = accuracy_metrics(run), accuracy_metrics(b)
        for name in sorted(set(new_acc) & set(base_acc)):
            print(
                f"  accuracy {name:19s} {base_acc[name]:10.2f} -> {new_acc[name]:10.2f}", file=file
            )


def finish(verdict):
    """Print the verdict as the last line and exit with its status."""
    print("\nVERDICT " + json.dumps(verdict, sort_keys=True))
    sys.exit(0 if verdict["status"] == "pass" else 1)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--freecad", help="FreeCAD GUI executable")
    parser.add_argument("--full", action="store_true", help="all jobs and sizes, with repeats")
    parser.add_argument(
        "--jobs",
        help=f"comma separated, from: {', '.join(list(jobs.JOBS) + list(jobs.DOCUMENT_JOBS))}",
    )
    parser.add_argument(
        "--sizes", help=f"comma separated, from: {', '.join(jobs.SIZES)} or a move count"
    )
    parser.add_argument(
        "--scenarios", help=f"comma separated, from: {', '.join(SCENARIOS + EXTRA_SCENARIOS)}"
    )
    parser.add_argument("--repeats", type=int, help="runs of each job, in separate processes")
    parser.add_argument("--window", help="main window size WxH")
    parser.add_argument("--quality", type=int, default=10, help="simulator accuracy 1..10")
    parser.add_argument(
        "--color-mode",
        choices=list(COLOR_MODES),
        default="operation",
        help="the colors of the stock, the Colors of the task panel (default: %(default)s)",
    )
    parser.add_argument("--view", default="ViewAxo", help="initial view, e.g. ViewAxo, ViewTop")
    parser.add_argument(
        "--timeout", type=float, default=120.0, help="seconds after which a scenario is stopped"
    )
    parser.add_argument(
        "--frame-timeout", type=float, default=600.0, help="max seconds to wait for one frame"
    )
    parser.add_argument(
        "--display", action="store_true", help="use the current display instead of Xvfb"
    )
    parser.add_argument("--images", help="directory to save a screenshot after each scenario")
    parser.add_argument(
        "--reference",
        default=REFERENCE_DIR,
        help="directory with correctness reference images (default: %(default)s)",
    )
    parser.add_argument(
        "--update-reference",
        action="store_true",
        help="save the correctness images as new reference images instead of comparing",
    )
    parser.add_argument(
        "--max-image-diff",
        type=float,
        default=0.01,
        help="max fraction of differing pixels for the correctness check (default: %(default)s)",
    )
    parser.add_argument(
        "--accuracy",
        action="store_true",
        help="measure the accuracy of every job against its exact material (CAM_simulator_accuracy)",
    )
    parser.add_argument(
        "--accuracy-tool",
        help="CAM_simulator_accuracy executable (default: from the tests next to --freecad)",
    )
    parser.add_argument(
        "--jobs-dir",
        help="directory to keep the jobs the simulator writes (CAM_simulator_accuracy input)",
    )
    parser.add_argument("--baseline", help="result file to compare this run against")
    parser.add_argument(
        "--fail-above",
        type=float,
        default=1.25,
        help="a metric regressed when it is this many times slower (default: %(default)s)",
    )
    parser.add_argument(
        "--noise-ms",
        type=float,
        default=2.0,
        help="differences up to this many ms are never a regression (default: %(default)s)",
    )
    parser.add_argument("--label", help="name of this run, e.g. 'baseline'")
    parser.add_argument("-o", "--output", help="result JSON file")
    parser.add_argument(
        "--compare", nargs=2, metavar=("BASE", "NEW"), help="compare two result files"
    )
    parser.add_argument("--summary", metavar="RESULT", help="print the summary of a result file")
    args = parser.parse_args()

    if args.compare:
        base, new = (json.load(open(p)) for p in args.compare)
        print_comparison(base, new)
        finish(evaluate(new, base, args.fail_above, args.noise_ms))
    if args.summary:
        data = json.load(open(args.summary))
        print_summary(data)
        finish(evaluate(data))
    if not args.freecad:
        parser.error("--freecad is required")
    if not args.display and not shutil.which("xvfb-run"):
        parser.error("xvfb-run not found, install Xvfb or use --display")

    defaults = FULL if args.full else QUICK
    for key in ("jobs", "sizes", "window", "repeats", "scenarios"):
        if getattr(args, key) is None:
            setattr(args, key, defaults[key])
    args.counts = defaults["counts"]
    args.playback_seconds = defaults["playback_seconds"]
    if isinstance(args.scenarios, str):
        args.scenarios = [s for s in args.scenarios.split(",") if s]
    args.window = [int(v) for v in args.window.lower().split("x")]
    args.freecad = os.path.abspath(args.freecad)
    args.reference = os.path.abspath(args.reference) if args.reference else None
    args.save_reference = args.reference if args.update_reference else None
    if args.save_reference:
        os.makedirs(args.save_reference, exist_ok=True)
    if args.images:
        args.images = os.path.abspath(args.images)
        os.makedirs(args.images, exist_ok=True)
    if args.accuracy and not args.accuracy_tool:
        build = os.path.dirname(os.path.dirname(args.freecad))
        name = "CAM_simulator_accuracy" + (".exe" if os.name == "nt" else "")
        candidates = [os.path.join(build, d, name) for d in ("tests", "bin")]
        args.accuracy_tool = next((c for c in candidates if os.path.exists(c)), None)
        if not args.accuracy_tool:
            parser.error(
                f"{name} not found in {build}/tests, build the tests or use --accuracy-tool"
            )
    if args.jobs_dir:
        args.jobs_dir = os.path.abspath(args.jobs_dir)
        os.makedirs(args.jobs_dir, exist_ok=True)
    baseline = json.load(open(args.baseline)) if args.baseline else None

    # a document job has one size, its own
    runs = []
    for job in args.jobs.split(","):
        if job not in jobs.JOBS and job not in jobs.DOCUMENT_JOBS:
            parser.error(f"unknown job {job}")
        sizes = [jobs.DOCUMENT_SIZE] if job in jobs.DOCUMENT_JOBS else args.sizes.split(",")
        runs += [{"job": job, "size": size} for size in sizes]
    data = {
        "label": args.label,
        "date": datetime.datetime.now().isoformat(timespec="seconds"),
        "git": git_info(),
        "host": {"cpu": cpu_name(), "cores": os.cpu_count(), "platform": platform.platform()},
        "config": {
            **{k: v for k, v in vars(args).items() if k not in ("compare", "summary")},
            "mode": "full" if args.full else "quick",
        },
        "runs": [],
    }

    start = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="camsim-bench-") as workdir:
        args.dump_dir = args.jobs_dir or (workdir if args.accuracy else None)
        for repeat in range(args.repeats):
            # quick mode runs everything in one process, full mode isolates every job
            batches = [[r] for r in runs] if args.full else [runs]
            for i, batch in enumerate(batches):
                names = ", ".join(f"{r['job']} {r['size']}" for r in batch)
                print(f"running {names} (repeat {repeat + 1}/{args.repeats}) ...", flush=True)
                for result in run_process(args, batch, workdir, f"{repeat}_{i}"):
                    result["repeat"] = repeat
                    data["runs"].append(result)
                data["wall_s"] = time.perf_counter() - start
                if args.output:
                    with open(args.output, "w") as f:
                        json.dump(data, f, indent=1)
        if args.accuracy:
            run_accuracy(args, data)
            data["wall_s"] = time.perf_counter() - start
            if args.output:
                with open(args.output, "w") as f:
                    json.dump(data, f, indent=1)

    print_summary(data)
    if baseline:
        print_comparison(baseline, data)
    finish(evaluate(data, baseline, args.fail_above, args.noise_ms))


if __name__ == "__main__":
    main()
