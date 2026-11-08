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
Gazebo launches get log_stream_ms:=200 (the controller appends its black-box log to the CSV every
200 ms); before the stop the runner calls the controller's ~/dump_log (10 s timeout, a failure is
only reported), and when the CSV is then missing or covers less than duration_s + hold_s it runs
`ros2 run cobot_ros2 cobot_logdump cobot_controller --out <CSV>` (appends the rows the controller
never exported) and judges the result.
Without cancel_after_s it passes on client exit 0 and the thresholds; with it, on client exit 3
(canceled as requested), last row in standby, max |qd| over the last rest_window_s (0.5 s) <
rest_qd_max_rad_s (0.01 rad/s) and peak |tau| / tau_max <= peak_tau_ratio (1); these optional
thresholds keep the defaults in parentheses (E2E-11b sets rest_qd_max_rad_s to 0.03 rad/s: the
CSV qd is the measured, quantized and filtered velocity, whose noise at rest in Gazebo has std
0.0047 rad/s per joint). cancel_after_s without waypoints fails.
A gazebo scenario with `bridge_steps: [<step>, ...]` (exclusive with `waypoints`) is driven
through the studio bridge: it launches with studio:=true, waits for standby, runs `ros2 run
cobot_ros2 cobot_bridge_cli cobot_controller <steps>` (`<out>` in a step = the scenario's output
directory; output in bridge.log) and stops the launch after it. With `panel_waypoints: FILE` the
runner runs `send_waypoints.py FILE` once the CLI printed its first line (`claim`, followed by a
pause step) and requires exit 1 with "studio is in control" in its output or in launch.log (the
controller logs why it rejected the goal). It passes on CLI exit
0, that refusal, a Run CSV with a row of every `expect.cmd` value, no holding row and a last row
in standby, and the CSV of the last `export:` step within the thresholds.
kind profiles runs `cobot_bench montecarlo` (runs seeds from seed) once per entry of `profiles`
(name -> config (optional, else the scenario's config), imperfections, hold: fail | report,
thresholds) into <out>/<id>/<name> and judges each run from the Summary CSV metric columns; with
hold: fail any held run fails the profile; with hold: report every held run, whatever its
hold_reason, counts in the printed hold rate (with the reasons), is not judged and never fails.
kind verify (feature verify) runs `cobot_bench verify --hardware H --algorithm A --task T --runs n
[--quick] --levels a,b --tune-seeds n --out <out>/<id>` (no `config` key: the controller file is the
hardware model's) and passes when every top-level key of the Verify report is in report.json,
`verdict` equals expect_verdict, `quick` is true and the sha256 of every versions.inputs file
matches its bytes. With lists (feature verify-analysis: `hardware` a list of model files,
`algorithm` a list of names) it is one call with both joined by commas (plus --seed0,
--sensitivity-seeds, --no-sensitivity when given) and passes when comparison.json has a row of every
`expect: {<model>__<algorithm>: verdict}` pair with that verdict and every text of
`expect_suggestions: {<pair>: [text]}` is in that pair's report.json suggestions.
Scenarios tagged nightly, realism2 or verify-nightly run only with --tag <that tag> or
--scenario. Exit 0 all pass, 1 any fails, 2 bad arguments.
"""

import argparse
import hashlib
import json
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
    read_torque_limits,
)

SCENARIO_DIR = REPO / "tests/e2e/scenarios"
THRESHOLD_KEYS = ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad")
OPT_IN_TAGS = {"nightly", "realism2", "verify-nightly"}  # never selected by a plain --all
GAZEBO_MARGIN_S = 5.0  # real-time factor below 1 and shutdown, on top of the trajectory time
GAZEBO_POLL_S = 0.1
GAZEBO_CAP = 3.0  # overall wait cap, as a multiple of the minimum wait
GAZEBO_TAIL_BYTES = 8192  # enough for the last complete CSV row
GAZEBO_STOP_S = 30.0  # graceful shutdown of the launch before it is killed
GAZEBO_STANDBY_S = 180.0  # launch until the controller reports standby
GAZEBO_ECHO_S = 10.0  # one `ros2 topic echo --once` call
GAZEBO_CLIENT_S = 600.0  # send_waypoints.py run, goal included
GAZEBO_STREAM_MS = 200  # log_stream_ms of the controller: run.csv grows while the run goes on
GAZEBO_DUMP_S = 10.0  # one ~/dump_log service call
GAZEBO_LOGDUMP_S = 60.0  # one cobot_logdump run
DUMP_LOG = ["ros2", "service", "call", "/cobot_controller/dump_log", "std_srvs/srv/Trigger"]
LOGDUMP = ["ros2", "run", "cobot_ros2", "cobot_logdump", "cobot_controller"]
BRIDGE_CLI = ["ros2", "run", "cobot_ros2", "cobot_bridge_cli", "cobot_controller"]
GAZEBO_BRIDGE_S = 900.0  # the whole cobot_bridge_cli run
PANEL_REFUSAL = "studio is in control"
STATUS_TOPIC = "/cobot_controller/status"
STATUS_TYPE = "std_msgs/msg/String"
# Canceled waypoint run (E2E-11b, design 2026-10-29 section 7): at rest and within the limits.
# Defaults of the optional scenario thresholds rest_window_s, rest_qd_max_rad_s, peak_tau_ratio.
CANCEL_DEFAULTS = {"rest_window_s": 0.5, "rest_qd_max_rad_s": 0.01, "peak_tau_ratio": 1.0}


# docs/conventions.md "Verify report"
VERIFY_KEYS = (
    *("schema", "disclaimer", "verdict", "quick", "runs", "seeds", "pass_count", "pass_rate"),
    *("wilson", "hold_rate", "worst", "tuning", "assumptions", "causes", "suggestions"),
    *("versions", "model", "algorithm", "task"),
)


def repo_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def urdf_of(s: dict) -> Path:
    """URDF named by the scenario's config (path relative to the config file)."""
    config = repo_path(s["config"])
    return config.parent / yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["urdf_path"]


def limits_of(s: dict) -> tuple[np.ndarray, np.ndarray]:
    """(peak, rated) torques of the scenario's config (cobot.limits, else the URDF effort)."""
    peak, rated = read_torque_limits(repo_path(s["config"]))
    return np.asarray(peak, dtype=float), np.asarray(rated, dtype=float)


def tau_max_of(s: dict) -> np.ndarray:
    """The controller's tau_max: the peak torque P of the scenario's config."""
    return limits_of(s)[0]


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


def judge_profile(name: str, p: dict, summary: Path) -> tuple[bool, str]:
    """One profile of kind profiles, judged run by run from its Summary CSV."""
    df = sensitivity.load_summary(summary)
    th = thresholds_of({"id": name, "thresholds": p["thresholds"]})
    held = df["held"] == 1
    problems = []
    if p["hold"] not in ("fail", "report"):
        raise ValueError(f"{name}: hold must be fail or report, got {p['hold']}")
    if p["hold"] == "fail" and held.any():
        problems.append(f"{int(held.sum())} held")
    judged = df[~held]
    bad = int((~sensitivity.passes(judged, th)).sum()) if len(judged) else 0
    if bad:
        problems.append(f"{bad} of {len(judged)} judged runs over the thresholds")
    detail = f"{name}: {len(df)} runs"
    if p["hold"] == "report":
        # Every hold counts, whatever its reason; reported, never failed.
        detail += f", hold rate {float(held.mean()):.3g}"
        if held.any():
            detail += f" ({', '.join(sorted(set(df.loc[held, 'hold_reason'].astype(str))))})"
    return not problems, detail + (f"; FAIL {'; '.join(problems)}" if problems else ", pass")


def run_profiles(s: dict, args: argparse.Namespace, out: Path, config: str) -> tuple[bool, str]:
    ok, lines = True, []
    for name, p in s["profiles"].items():
        runs = ("--runs", str(s["runs"]), "--seed0", str(s["seed"]))
        imperfections = str(repo_path(p["imperfections"]))
        cfg = str(repo_path(p["config"])) if "config" in p else config
        cmd = ("montecarlo", "--config", cfg, "--imperfections", imperfections, *runs)
        try:
            bench(args, *cmd, "--out", str(out / name))
            good, line = judge_profile(name, p, out / name / "summary.csv")
        except (OSError, ValueError, KeyError, RuntimeError) as err:
            good, line = False, f"{name}: FAIL {err}"
        ok, lines = ok and good, [*lines, line]
    return ok, "\n    ".join(lines)


def judge_run(s: dict, csv: Path, out: Path, report: bool) -> tuple[bool, str]:
    peak, rated = limits_of(s)
    m = compute_metrics(load_run(csv), peak, meta=load_meta(csv), rated_torque=rated)
    failures = check(m, thresholds_of(s))
    if report:
        # Imported here: the arm64 image has no matplotlib and runs with --no-report.
        from cobot_eval import report as report_mod

        (out / "scenario.yaml").write_text(yaml.safe_dump(s), encoding="utf-8")
        report_mod.main(
            ["--csv", str(csv), "--thresholds", str(out / "scenario.yaml"), "--out", str(out)]
            + ["--urdf", str(urdf_of(s)), "--config", str(repo_path(s["config"]))]
            + ["--title", f"{s['id']} report"]
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


def bridge_steps(s: dict, out: Path) -> list[str]:
    return [str(step).replace("<out>", str(out)) for step in s["bridge_steps"]]


def panel_check(s: dict, cli: subprocess.Popen, bridge_log: Path, out: Path) -> str:
    """Once the CLI printed its first step, the RViz panel client must be refused; error or ''."""
    while cli.poll() is None and not bridge_log.read_text(encoding="utf-8").strip():
        time.sleep(GAZEBO_POLL_S)
    cmd = ["ros2", "run", "cobot_ros2", "send_waypoints.py", str(repo_path(s["panel_waypoints"]))]
    try:
        panel = subprocess.run(
            cmd, capture_output=True, text=True, timeout=GAZEBO_CLIENT_S, check=False
        )
    except subprocess.TimeoutExpired:
        return f"panel client still running after {GAZEBO_CLIENT_S:g} s"
    (out / "panel.log").write_text(panel.stdout + panel.stderr, encoding="utf-8")
    # The action protocol carries no rejection reason: the controller logs it (launch.log).
    launch_log = out / "launch.log"
    said = panel.stdout + panel.stderr
    said += launch_log.read_text(encoding="utf-8", errors="replace") if launch_log.is_file() else ""
    if panel.returncode != 1 or PANEL_REFUSAL not in said:
        return f"panel client exit {panel.returncode}, expected 1 with '{PANEL_REFUSAL}'"
    return ""


def drive_bridge(s: dict, proc: subprocess.Popen, out: Path) -> str:
    """Waits for standby, runs cobot_bridge_cli (and the panel check); error text or ''."""
    if reason := wait_standby(proc):
        return f"{reason} (see {out / 'launch.log'})"
    bridge_log = out / "bridge.log"
    with bridge_log.open("w", encoding="utf-8") as log:
        # Relative paths in the steps are relative to the repository root.
        steps = [*BRIDGE_CLI, *bridge_steps(s, out)]
        cli = subprocess.Popen(steps, stdout=log, stderr=log, cwd=REPO)
        try:
            panel = panel_check(s, cli, bridge_log, out) if s.get("panel_waypoints") else ""
            code = cli.wait(timeout=GAZEBO_BRIDGE_S)
        except subprocess.TimeoutExpired:
            cli.kill()
            cli.wait()
            return f"cobot_bridge_cli still running after {GAZEBO_BRIDGE_S:g} s"
    if code != 0:
        return f"cobot_bridge_cli exit {code} (see {bridge_log})"
    return panel


def judge_bridge(s: dict, csv: Path, out: Path, report: bool) -> tuple[bool, str]:
    """Run CSV: every expected cmd, no holding row, last row standby; then the last segment."""
    df = load_run(csv)
    mode, cmd = df["mode"].astype(str), df["cmd"].astype(str)
    problems = [f"no {c} row" for c in s.get("expect", {}).get("cmd", []) if not (cmd == c).any()]
    if (mode == "holding").any():
        problems.append(f"{int((mode == 'holding').sum())} holding rows")
    if mode.iloc[-1] != "standby":
        problems.append(f"last mode {mode.iloc[-1]}, not standby")
    exports = [st.split(":", 1)[1] for st in bridge_steps(s, out) if st.startswith("export:")]
    if problems or not exports:
        return False, "; ".join(problems) or "no export step to judge"
    ok, detail = judge_run(s, Path(exports[-1]), out, report)
    return ok, f"segment: {detail}"


def dump_log(proc: subprocess.Popen, out: Path) -> None:
    """Asks the running controller to write its black-box log to the CSV; failure only noted."""
    if proc.poll() is not None:
        return
    with (out / "dump_log.log").open("w", encoding="utf-8") as log:
        try:
            done = subprocess.run(DUMP_LOG, stdout=log, stderr=log, timeout=GAZEBO_DUMP_S)
            problem = f"exit {done.returncode}" if done.returncode != 0 else ""
        except subprocess.TimeoutExpired:
            problem = f"no answer within {GAZEBO_DUMP_S:g} s"
    if problem:
        print(f"note: ~/dump_log failed ({problem}, see {out / 'dump_log.log'})", file=sys.stderr)


def logdump(csv: Path, out: Path) -> None:
    """Appends what the controller never exported to csv with cobot_logdump (after the stop)."""
    with (out / "logdump.log").open("w", encoding="utf-8") as log:
        try:
            done = subprocess.run(
                [*LOGDUMP, "--out", str(csv)], stdout=log, stderr=log, timeout=GAZEBO_LOGDUMP_S
            )
            problem = f"exit {done.returncode}" if done.returncode != 0 else ""
        except subprocess.TimeoutExpired:
            problem = f"still running after {GAZEBO_LOGDUMP_S:g} s"
    if problem:
        print(f"note: cobot_logdump {problem} (see {out / 'logdump.log'})", file=sys.stderr)


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
    studio = bool(s.get("bridge_steps"))
    if s.get("cancel_after_s") is not None and not waypoints:
        return False, "cancel_after_s needs waypoints"
    if studio and waypoints:
        return False, "bridge_steps and waypoints are exclusive"
    config = repo_path(s["config"])
    traj = yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["trajectory"]
    run_s = float(traj["duration_s"]) + float(traj["hold_s"])
    wait_s = run_s + GAZEBO_MARGIN_S
    csv = out / "run.csv"
    csv.unlink(missing_ok=True)
    cmd = ["ros2", "launch", "cobot_sim", "sim.launch.py", "gui:=false"]
    cmd += ["controller:=cobot_controller", f"config_path:={config}"]
    cmd += [f"imperfections_path:={repo_path(s['imperfections'])}", f"seed:={int(s['seed'])}"]
    cmd += [f"log_path:={csv}", f"log_stream_ms:={GAZEBO_STREAM_MS}"]
    cmd += ["command_source:=external"] if waypoints else []
    cmd += ["studio:=true"] if studio else []
    with (out / "launch.log").open("w", encoding="utf-8") as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if studio:
                failed = drive_bridge(s, proc, out)
            elif waypoints:
                failed = drive_waypoints(s, proc, csv, out, float(traj["hold_s"]))
            elif (code := wait_run(proc, csv, wait_s, round(run_s * 1e9))) is not None:
                failed = f"launch exited with {code} before the run ended (see {log.name})"
            else:
                failed = ""
        finally:
            dump_log(proc, out)
            stop_launch(proc)
    if failed:
        return False, failed
    if not csv.is_file() or csv_span_ns(csv) < round(run_s * 1e9):
        logdump(csv, out)
    if not csv.is_file():
        return False, f"no CSV at {csv}"
    if studio:
        return judge_bridge(s, csv, out, report)
    if s.get("cancel_after_s") is not None:
        return judge_cancel(s, csv)
    return judge_run(s, csv, out, report)


def run_verify_lists(s: dict, args: argparse.Namespace, out: Path) -> tuple[bool, str]:
    hardware = ",".join(str(repo_path(h)) for h in s["hardware"])
    cmd = ["verify", "--hardware", hardware, "--algorithm", ",".join(s["algorithm"])]
    cmd += ["--task", str(repo_path(s["task"])), "--runs", str(s["runs"])]
    cmd += ["--quick"] if s.get("quick") else []
    for key, name in (("seed0", "--seed0"), ("sensitivity_seeds", "--sensitivity-seeds")):
        cmd += [name, str(s[key])] if key in s else []
    cmd += ["--no-sensitivity"] if s.get("no_sensitivity") else []
    if "levels" in s:
        cmd += ["--levels", ",".join(str(v) for v in s["levels"])]
    if "tune_seeds" in s:
        cmd += ["--tune-seeds", str(s["tune_seeds"])]
    bench(args, *cmd, "--out", str(out))
    rows = json.loads((out / "comparison.json").read_text(encoding="utf-8"))["rows"]
    verdicts = {f"{r['model']}__{r['algorithm']}": r for r in rows}
    problems, seen = [], []
    for pair, want in s["expect"].items():
        row = verdicts.get(pair)
        if row is None:
            problems.append(f"{pair}: no row in comparison.json")
            continue
        seen.append(f"{pair} {row['verdict']} {row.get('pass_count')}/{row.get('runs')}")
        if row["verdict"] != want:
            problems.append(f"{pair}: verdict {row['verdict']}, expected {want}")
    for pair, texts in (s.get("expect_suggestions") or {}).items():
        path = out / pair / "report.json"
        got = []
        if path.is_file():
            got = json.loads(path.read_text(encoding="utf-8")).get("suggestions", [])
        problems += [f"{pair}: no suggestion {text!r}" for text in texts if text not in got]
    return not problems, "; ".join(seen + problems)


def run_verify(s: dict, args: argparse.Namespace, out: Path) -> tuple[bool, str]:
    if isinstance(s["hardware"], list):
        return run_verify_lists(s, args, out)
    cmd = ["verify", "--hardware", str(repo_path(s["hardware"])), "--algorithm", s["algorithm"]]
    cmd += ["--task", str(repo_path(s["task"])), "--runs", str(s["runs"])]
    cmd += ["--quick"] if s.get("quick") else []
    if "levels" in s:
        cmd += ["--levels", ",".join(str(v) for v in s["levels"])]
    if "tune_seeds" in s:
        cmd += ["--tune-seeds", str(s["tune_seeds"])]
    bench(args, *cmd, "--out", str(out))
    report = json.loads((out / "report.json").read_text(encoding="utf-8"))
    problems = [f"no {k} in report.json" for k in VERIFY_KEYS if k not in report]
    if report.get("verdict") != s["expect_verdict"]:
        problems.append(f"verdict {report.get('verdict')}, expected {s['expect_verdict']}")
    if report.get("quick") is not True:
        problems.append(f"quick {report.get('quick')}, expected true")
    inputs = report.get("versions", {}).get("inputs") or []
    if not inputs:
        problems.append("no file in versions.inputs")
    for f in inputs:
        if hashlib.sha256(Path(f["path"]).read_bytes()).hexdigest() != f["sha256"]:
            problems.append(f"sha256 of {f['path']} does not match")
    verdict, passed = report.get("verdict"), report.get("pass_count")
    detail = f"verdict {verdict}, {passed}/{report.get('runs')} pass"
    return not problems, detail + (f"; {'; '.join(problems)}" if problems else "")


def run_scenario(s: dict, args: argparse.Namespace) -> tuple[bool, str]:
    out = args.out / str(s["id"])
    out.mkdir(parents=True, exist_ok=True)
    if s.get("runner") == "gazebo":
        return run_gazebo(s, out, not args.no_report)
    if s["kind"] == "verify":
        return run_verify(s, args, out)
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
    if s["kind"] == "profiles":
        return run_profiles(s, args, out, config)
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
        if (args.tag in s.get("tags", []) if args.tag else not OPT_IN_TAGS & set(s.get("tags", [])))
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
