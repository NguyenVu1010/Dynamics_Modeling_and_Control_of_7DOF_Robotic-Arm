"""End-to-end scenario runner (design 11.3): runs cobot_bench and judges the result.

Usage: python tools/run_e2e.py (--scenario YAML | --all [--tag T]) [--bench PATH] --out DIR
       [--no-report]
Scenario paths are relative to the repository root. A bench run scenario may name a waypoint file
(`waypoints: <path>`, passed as `cobot_bench run --waypoints`); its optional threshold
`stop_pos_err_m` is judged from the run's meta file. kind: run (thresholds on the Run CSV),
determinism (two runs, identical bytes), safety (expect.mode_reached, |tau| <= tau_max and
finite), timing (p95 of `cobot_bench timing` <= thresholds.p95_step_us), montecarlo (share of
runs in the summary meeting the thresholds >= thresholds.pass_rate), sweep (bench completes and
the sensitivity report with hardware_requirements.md is produced). --bench is needed only by
bench scenarios. runner: gazebo scenarios (kind run) launch headless Gazebo with
cobot_controller, the scenario's profile and seed (needs a sourced colcon install of cobot_sim
and cobot_ros2), wait at least duration_s + hold_s + 5 s after the controller created its CSV
(or after launching, if it never does), then keep waiting until the CSV covers duration_s +
hold_s of sim time (Gazebo may run slower than real time), at most 3x that minimum wait after
the CSV appeared, stop the launch and judge the CSV; they are skipped unless --tag gazebo.
Scenarios tagged nightly run only with --tag nightly or --scenario. Exit 0 all pass, 1 any
fails, 2 bad arguments.
"""

import argparse
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "eval"))

from cobot_eval import sensitivity  # noqa: E402
from cobot_eval.metrics import (  # noqa: E402
    Thresholds,
    check,
    compute_metrics,
    joint_array,
    load_meta,
    load_run,
    read_tau_max,
)

SCENARIO_DIR = REPO / "tests/e2e/scenarios"
THRESHOLD_KEYS = ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad")
GAZEBO_MARGIN_S = 5.0  # real-time factor below 1 and shutdown, on top of the trajectory time
GAZEBO_POLL_S = 0.1
GAZEBO_CAP = 3.0  # overall wait cap, as a multiple of the minimum wait
GAZEBO_TAIL_BYTES = 8192  # enough for the last complete CSV row
GAZEBO_STOP_S = 30.0  # graceful shutdown of the launch before it is killed


def repo_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def urdf_of(s: dict) -> Path:
    """URDF named by the scenario's config (path relative to the config file)."""
    config = repo_path(s["config"])
    return config.parent / yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["urdf_path"]


def tau_max_of(s: dict) -> np.ndarray:
    return np.asarray(read_tau_max(urdf_of(s)), dtype=float)


def bench(args: argparse.Namespace, *cmd: str) -> subprocess.CompletedProcess:
    if args.bench is None:
        raise RuntimeError("--bench is required for bench scenarios")
    proc = subprocess.run([str(args.bench), *cmd], capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"bench exit {proc.returncode}: {proc.stderr.strip()}")
    return proc


def thresholds_of(s: dict) -> Thresholds:
    th = s["thresholds"]
    stop = th.get("stop_pos_err_m")  # optional: judged from the run's meta file
    return Thresholds(
        *(float(th[k]) for k in THRESHOLD_KEYS),
        stop_pos_err_m=None if stop is None else float(stop),
    )


def sensitivity_report(s: dict, summary: Path, out: Path) -> None:
    (out / "scenario.yaml").write_text(yaml.safe_dump(s), encoding="utf-8")
    args = ["--summary", str(summary), "--thresholds", str(out / "scenario.yaml")]
    if sensitivity.main([*args, "--out", str(out / "report")]) != 0:
        raise RuntimeError("sensitivity report failed")


def run_nightly(s: dict, args: argparse.Namespace, out: Path, config: str) -> tuple[bool, str]:
    if s["kind"] == "montecarlo":
        imperfections = str(repo_path(s["imperfections"]))
        runs = ("--runs", str(s["runs"]), "--seed0", str(s["seed"]))
        bench(
            args,
            "montecarlo",
            "--config",
            config,
            "--imperfections",
            imperfections,
            *runs,
            "--out",
            str(out),
        )
    else:
        bench(
            args,
            "sweep",
            "--config",
            config,
            "--sweep",
            str(repo_path(s["sweep"])),
            "--out",
            str(out),
        )
    summary = sensitivity.load_summary(out / "summary.csv")
    if args.no_report:
        # The tables need no matplotlib; only the PNGs are skipped.
        sensitivity.write_tables(summary, thresholds_of(s), out / "report")
    else:
        sensitivity_report(s, out / "summary.csv", out)
    if s["kind"] == "sweep":
        produced = (out / "report/hardware_requirements.md").is_file()
        return produced, f"{len(summary)} runs, report {'produced' if produced else 'missing'}"
    rate = float(sensitivity.passes(summary, thresholds_of(s)).mean())
    limit = float(s["thresholds"]["pass_rate"])
    return rate >= limit, f"pass rate {rate:.4g} over {len(summary)} runs (limit {limit:g})"


def judge_run(s: dict, csv: Path, out: Path, report: bool) -> tuple[bool, str]:
    m = compute_metrics(load_run(csv), tau_max_of(s), meta=load_meta(csv))
    failures = check(m, thresholds_of(s))
    if report:
        # Imported here: the arm64 image has no matplotlib and runs with --no-report.
        from cobot_eval import report as report_mod

        (out / "scenario.yaml").write_text(yaml.safe_dump(s), encoding="utf-8")
        report_mod.main(
            ["--csv", str(csv), "--thresholds", str(out / "scenario.yaml"), "--out", str(out)]
            + ["--urdf", str(urdf_of(s)), "--title", f"{s['id']} report"]
        )
    pos, rot = m.final_pos_err_m * 1e3, np.degrees(m.final_rot_err_rad)
    good = f"pos {pos:.3f} mm, rot {rot:.3f} deg, rms {m.rms_joint_err_rad:.4g} rad"
    return not failures, "; ".join(failures) or good


def judge_safety(s: dict, csv: Path) -> tuple[bool, str]:
    df = load_run(csv)
    want, tau = s["expect"]["mode_reached"], joint_array(df, "tau")
    problems = [] if (df["mode"].astype(str) == want).any() else [f"{want} never reached"]
    if not np.all(np.isfinite(tau)):
        problems.append("non-finite torque")
    elif np.any(np.abs(tau) > tau_max_of(s)):
        problems.append(f"|tau| / tau_max peaks at {np.max(np.abs(tau) / tau_max_of(s)):.4g}")
    return not problems, "; ".join(problems) or f"{want} reached, torque within limits"


def stop_launch(proc: subprocess.Popen) -> None:
    """SIGINT (as Ctrl-C) so the controller flushes its CSV, then kill what is left."""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=GAZEBO_STOP_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    try:
        os.killpg(proc.pid, signal.SIGKILL)  # stragglers of the launch's process group
    except (ProcessLookupError, PermissionError):
        pass


def csv_t_ns(line: bytes) -> int | None:
    """t_ns (first column) of a complete data row, None for the header or a partial row."""
    try:
        return int(line.split(b",", 1)[0])
    except ValueError:
        return None


def csv_span_ns(csv: Path) -> int:
    """Last row's t_ns minus the first row's t_ns of a CSV being written (0 if too short)."""
    try:
        with csv.open("rb") as f:
            lines = f.read(GAZEBO_TAIL_BYTES).split(b"\n")
            first = csv_t_ns(lines[1]) if len(lines) > 2 else None
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - GAZEBO_TAIL_BYTES))
            tail = f.read().split(b"\n")[:-1]  # rows end with a newline: drop the partial one
    except OSError:
        return 0
    last = csv_t_ns(tail[-1]) if tail else None
    return 0 if first is None or last is None else last - first


def wait_run(proc: subprocess.Popen, csv: Path, wait_s: float, run_ns: int) -> int | None:
    """Exit code if the launch ends early, else None once wait_s has passed since the CSV
    appeared (the controller is active from then on) and the CSV covers run_ns of sim time,
    or GAZEBO_CAP * wait_s has passed since then. Without a CSV the clock runs from the start
    and the launch is stopped after wait_s."""
    base = time.monotonic()
    csv_seen = False
    while True:
        if (code := proc.poll()) is not None:
            return code
        now = time.monotonic()
        if not csv_seen and csv.exists():
            csv_seen, base = True, now
        if not csv_seen:
            if now >= base + wait_s:
                return None
        elif now >= base + GAZEBO_CAP * wait_s:
            return None
        elif now >= base + wait_s and csv_span_ns(csv) >= run_ns:
            return None
        time.sleep(GAZEBO_POLL_S)


def run_gazebo(s: dict, out: Path, report: bool) -> tuple[bool, str]:
    if s["kind"] != "run":
        return False, f"gazebo runner supports kind run only, got {s['kind']}"
    config = repo_path(s["config"])
    traj = yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["trajectory"]
    run_s = float(traj["duration_s"]) + float(traj["hold_s"])
    wait_s = run_s + GAZEBO_MARGIN_S
    csv = out / "run.csv"
    csv.unlink(missing_ok=True)
    cmd = ["ros2", "launch", "cobot_sim", "sim.launch.py", "gui:=false"]
    cmd += ["controller:=cobot_controller", f"config_path:={config}"]
    cmd += [f"imperfections_path:={repo_path(s['imperfections'])}", f"seed:={int(s['seed'])}"]
    cmd += [f"log_path:={csv}"]
    with (out / "launch.log").open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = wait_run(proc, csv, wait_s, round(run_s * 1e9))
        finally:
            stop_launch(proc)
    if code is not None:
        return False, f"launch exited with {code} before the run ended (see {out / 'launch.log'})"
    if not csv.is_file():
        return False, f"no CSV at {csv}"
    return judge_run(s, csv, out, report)


def run_scenario(s: dict, args: argparse.Namespace) -> tuple[bool, str]:
    out = args.out / str(s["id"])
    out.mkdir(parents=True, exist_ok=True)
    if s.get("runner") == "gazebo":
        return run_gazebo(s, out, not args.no_report)
    config = str(repo_path(s["config"]))
    if s["kind"] == "timing":
        text = bench(args, "timing", "--config", config, "--steps", str(s["steps"])).stdout
        match = re.search(r"p95_us=(\S+)", text)
        if match is None:
            return False, f"no p95_us in output: {text.strip()}"
        limit = float(s["thresholds"]["p95_step_us"])
        return float(match.group(1)) <= limit, f"{text.strip()} (limit p95 {limit:g} us)"
    if s["kind"] in ("montecarlo", "sweep"):
        return run_nightly(s, args, out, config)
    csvs = [out / f"run{k}.csv" for k in range(1, 3 if s["kind"] == "determinism" else 2)]
    for csv in csvs:
        imperfections = str(repo_path(s["imperfections"]))
        seed_out = ("--seed", str(s["seed"]), "--out", str(csv))
        if s.get("waypoints"):
            seed_out += ("--waypoints", str(repo_path(s["waypoints"])))
        bench(args, "run", "--config", config, "--imperfections", imperfections, *seed_out)
    if s["kind"] == "determinism":
        same = csvs[0].read_bytes() == csvs[1].read_bytes()
        return same, "identical bytes" if same else "the two runs differ"
    if s["kind"] == "safety":
        return judge_safety(s, csvs[0])
    if s["kind"] == "run":
        return judge_run(s, csvs[0], out, not args.no_report)
    return False, f"unknown kind {s['kind']}"


def evaluate(s: dict, args: argparse.Namespace) -> str:
    if s.get("runner") == "gazebo" and args.all and args.tag != "gazebo":
        return "SKIP   gazebo runner (use --tag gazebo)"
    try:
        ok, detail = run_scenario(s, args)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as err:
        ok, detail = False, f"error: {err}"
    return f"{'PASS' if ok else 'FAIL'}   {detail}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="run_e2e", description=__doc__)
    which = parser.add_mutually_exclusive_group(required=True)
    which.add_argument("--scenario", type=Path)
    which.add_argument("--all", action="store_true")
    parser.add_argument("--tag", default=None)
    parser.add_argument("--bench", type=Path, default=None)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-report", action="store_true")
    args = parser.parse_args(argv)
    args.bench = args.bench.resolve() if args.bench else None
    args.out = args.out.resolve()

    paths = [args.scenario] if args.scenario else sorted(SCENARIO_DIR.glob("*.yaml"))
    try:
        scenarios = [yaml.safe_load(p.read_text(encoding="utf-8")) for p in paths]
    except (OSError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 2
    scenarios = [
        s
        for s in scenarios
        if (args.tag in s.get("tags", []) if args.tag else "nightly" not in s.get("tags", []))
        or args.scenario
    ]
    if not scenarios:
        print("error: no scenario selected", file=sys.stderr)
        return 2
    rows = [(str(s.get("id")), str(s.get("kind")), evaluate(s, args)) for s in scenarios]
    print(f"{'scenario':<10} {'kind':<12} result")
    for sid, kind, result in rows:
        print(f"{sid:<10} {kind:<12} {result}")
    return 1 if any(r.startswith("FAIL") for _, _, r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
