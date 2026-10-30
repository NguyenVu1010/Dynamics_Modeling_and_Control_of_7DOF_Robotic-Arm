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
the CSV appeared, stop the launch and judge the CSV; they are skipped unless --tag gazebo. A
gazebo scenario with `waypoints: FILE` launches with command_source:=external instead, waits for
`standby` on /cobot_controller/status, runs `ros2 run cobot_ros2 send_waypoints.py FILE
[--cancel-after cancel_after_s]`, lets the CSV cover hold_s more of sim time and stops the launch.
Without cancel_after_s it passes on client exit 0 and the thresholds; with it, on client exit 3
(canceled as requested), last row in standby, max |qd| over the last rest_window_s (0.5 s) <
rest_qd_max_rad_s (0.01 rad/s) and peak |tau| / tau_max <= peak_tau_ratio (1); these optional
thresholds keep the defaults in parentheses (E2E-11b sets rest_qd_max_rad_s to 0.03 rad/s: the
CSV qd is the measured, quantized and filtered velocity, whose noise at rest in Gazebo has std
0.0047 rad/s per joint). cancel_after_s without waypoints fails.
Scenarios tagged nightly run only with --tag nightly or --scenario. Exit 0 all pass, 1 any
fails, 2 bad arguments.
"""

import argparse
import math
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
sys.path.insert(0, str(REPO / "src/eval"))

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
GAZEBO_STANDBY_S = 180.0  # launch until the controller reports standby
GAZEBO_ECHO_S = 10.0  # one `ros2 topic echo --once` call
GAZEBO_CLIENT_S = 600.0  # send_waypoints.py run, goal included
STATUS_TOPIC = "/cobot_controller/status"
STATUS_TYPE = "std_msgs/msg/String"
# Canceled waypoint run (E2E-11b, design 2026-10-29 section 7): at rest and within the limits.
# Defaults of the optional scenario thresholds rest_window_s, rest_qd_max_rad_s, peak_tau_ratio.
CANCEL_DEFAULTS = {"rest_window_s": 0.5, "rest_qd_max_rad_s": 0.01, "peak_tau_ratio": 1.0}


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
    parsed = Thresholds(
        *(float(th[k]) for k in THRESHOLD_KEYS),
        stop_pos_err_m=None if stop is None else float(stop),
    )
    for key, value in vars(parsed).items():
        if value is not None and not math.isfinite(value):
            raise ValueError(f"{s.get('id', '?')}: threshold {key} is not finite")
    return parsed


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


def wait_standby(proc: subprocess.Popen) -> str | None:
    """None once the controller's ~/status reads standby, else why it never did."""
    echo = ["ros2", "topic", "echo", "--once", "--field", "data", STATUS_TOPIC, STATUS_TYPE]
    deadline = time.monotonic() + GAZEBO_STANDBY_S
    while time.monotonic() < deadline:
        if (code := proc.poll()) is not None:
            return f"launch exited with {code} before standby"
        try:
            proc_echo = subprocess.run(echo, capture_output=True, text=True, timeout=GAZEBO_ECHO_S)
        except subprocess.TimeoutExpired:
            continue
        if "standby" in proc_echo.stdout.split():
            return None
        time.sleep(GAZEBO_POLL_S)
    return f"no standby on {STATUS_TOPIC} within {GAZEBO_STANDBY_S:g} s"


def settle(proc: subprocess.Popen, csv: Path, hold_s: float) -> None:
    """Until the CSV covers hold_s more of sim time, the launch ends or GAZEBO_CAP * hold_s."""
    target = csv_span_ns(csv) + round(hold_s * 1e9)
    end = time.monotonic() + GAZEBO_CAP * hold_s
    while time.monotonic() < end and proc.poll() is None and csv_span_ns(csv) < target:
        time.sleep(GAZEBO_POLL_S)


def drive_waypoints(s: dict, proc: subprocess.Popen, csv: Path, out: Path, hold_s: float) -> str:
    """Waits for standby, runs the client and lets the run settle; error text or ''."""
    if reason := wait_standby(proc):
        return f"{reason} (see {out / 'launch.log'})"
    cmd = ["ros2", "run", "cobot_ros2", "send_waypoints.py", str(repo_path(s["waypoints"]))]
    if s.get("cancel_after_s") is not None:
        cmd += ["--cancel-after", str(float(s["cancel_after_s"]))]
    with (out / "client.log").open("w", encoding="utf-8") as log:
        try:
            client = subprocess.run(cmd, stdout=log, stderr=log, timeout=GAZEBO_CLIENT_S)
        except subprocess.TimeoutExpired:
            return f"client still running after {GAZEBO_CLIENT_S:g} s"
    want = 0 if s.get("cancel_after_s") is None else 3
    if client.returncode != want:
        return f"client exit {client.returncode}, expected {want} (see {out / 'client.log'})"
    if (ended := proc.poll()) is not None:
        return f"launch exited with {ended} during the goal (see {out / 'launch.log'})"
    settle(proc, csv, hold_s)
    return ""


def judge_cancel(s: dict, csv: Path) -> tuple[bool, str]:
    th = {k: float(s.get("thresholds", {}).get(k, v)) for k, v in CANCEL_DEFAULTS.items()}
    df = load_run(csv)
    t_ns, mode = df["t_ns"].to_numpy(dtype=np.int64), df["mode"].astype(str).to_numpy()
    rest = t_ns >= t_ns[-1] - round(th["rest_window_s"] * 1e9)
    qd = float(np.max(np.abs(joint_array(df, "qd")[rest])))
    ratio = float(np.max(np.abs(joint_array(df, "tau")) / tau_max_of(s)))
    problems = [] if mode[-1] == "standby" else [f"last mode {mode[-1]}, not standby"]
    if not qd < th["rest_qd_max_rad_s"]:
        problems.append(f"max |qd| {qd:.4g} rad/s over the last {th['rest_window_s']:g} s")
    if not ratio <= th["peak_tau_ratio"]:
        problems.append(f"|tau| / tau_max peaks at {ratio:.4g}")
    return not problems, "; ".join(problems) or f"stopped, max |qd| {qd:.3g} rad/s"


def run_gazebo(s: dict, out: Path, report: bool) -> tuple[bool, str]:
    if s["kind"] != "run":
        return False, f"gazebo runner supports kind run only, got {s['kind']}"
    waypoints = bool(s.get("waypoints"))
    if s.get("cancel_after_s") is not None and not waypoints:
        return False, "cancel_after_s needs waypoints"
    config = repo_path(s["config"])
    traj = yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["trajectory"]
    run_s = float(traj["duration_s"]) + float(traj["hold_s"])
    wait_s = run_s + GAZEBO_MARGIN_S
    csv = out / "run.csv"
    csv.unlink(missing_ok=True)
    cmd = ["ros2", "launch", "cobot_sim", "sim.launch.py", "gui:=false"]
    cmd += ["controller:=cobot_controller", f"config_path:={config}"]
    cmd += [f"imperfections_path:={repo_path(s['imperfections'])}", f"seed:={int(s['seed'])}"]
    cmd += [f"log_path:={csv}"] + (["command_source:=external"] if waypoints else [])
    with (out / "launch.log").open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if waypoints:
                failed = drive_waypoints(s, proc, csv, out, float(traj["hold_s"]))
            elif (code := wait_run(proc, csv, wait_s, round(run_s * 1e9))) is not None:
                failed = f"launch exited with {code} before the run ended (see {log.name})"
            else:
                failed = ""
        finally:
            stop_launch(proc)
    if failed:
        return False, failed
    if not csv.is_file():
        return False, f"no CSV at {csv}"
    if s.get("cancel_after_s") is not None:
        return judge_cancel(s, csv)
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
