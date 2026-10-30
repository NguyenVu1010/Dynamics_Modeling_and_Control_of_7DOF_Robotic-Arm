"""tools/run_e2e.py, Gazebo scenarios with `waypoints:` and `cancel_after_s:` (feature
waypoint-ros), against a fake `ros2` on PATH (fake_ros2_external.py, no Gazebo, no rclpy).

Contract pinned here:
- a gazebo scenario with `waypoints: FILE` launches as today (config_path, imperfections_path,
  seed, log_path) plus `command_source:=external`; scenarios without it get no command_source;
- the runner waits for `standby` on the controller's `~/status` topic (/cobot_controller/status,
  read with `ros2 topic echo`), then runs the client as
  `ros2 run cobot_ros2 send_waypoints.py <absolute FILE> [--cancel-after S]` while the launch is
  up, stops the launch once the client has returned, and judges the CSV;
- without cancel_after_s: PASS needs client exit 0 and the scenario thresholds on the CSV
  (same metrics as bench);
- with cancel_after_s: PASS needs client exit 3 (canceled as requested), last CSV row in mode
  standby, max |qd| over the last 0.5 s of the CSV < 0.01 rad/s and peak |tau|/tau_max <= 1
  (defaults; a scenario may set rest_window_s, rest_qd_max_rad_s, peak_tau_ratio);
- cancel_after_s without waypoints is a FAIL; a launch that dies before standby is a FAIL
  reported promptly, and the client is never run.
"""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
REALISTIC = REPO / "config/imperfections/realistic.yaml"
# E2E-02 thresholds (E2E-10 and E2E-11 use them).
G2 = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}
STATUS_TOPIC = "/cobot_controller/status"


@pytest.fixture
def ros2(tmp_path):
    """Directory holding an executable fake `ros2`; returns (bin dir, JSON-lines call log)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "ros2"
    source = (Path(__file__).parent / "fake_ros2_external.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return bindir, tmp_path / "ros2_calls.jsonl"


def short_config(tmp_path):
    """default.yaml with duration_s 0.5 and hold_s 0.5, paths made absolute."""
    path = tmp_path / "short.yaml"
    if path.exists():
        return path
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    cobot = data["cobot"]
    cobot["urdf_path"] = str((CONFIG.parent / cobot["urdf_path"]).resolve())
    cobot["log"]["output_dir"] = str(tmp_path / "results")
    cobot["trajectory"]["duration_s"] = 0.5
    cobot["trajectory"]["hold_s"] = 0.5
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def waypoint_file(tmp_path):
    path = tmp_path / "wp.yaml"
    first = {"position_m": [-0.5, -0.6, 0.9], "rpy_deg": [0, 0, 90]}
    second = {"position_m": [-0.3, -0.6, 0.7], "rpy_deg": [0, 0, 90]}
    data = {
        "frame": "base_link",
        "waypoints": [
            first | {"duration_s": 0, "stop": False},
            second | {"duration_s": 4.0, "stop": True},
        ],
    }
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def scenario(tmp_path, *, waypoints=True, cancel_after_s=None, thresholds=None):
    data = {"id": "T-wp", "tags": ["gazebo"], "runner": "gazebo", "kind": "run"}
    data |= {"config": str(short_config(tmp_path)), "imperfections": str(REALISTIC), "seed": 7}
    data["thresholds"] = thresholds or G2
    if waypoints:
        data["waypoints"] = str(waypoint_file(tmp_path))
    if cancel_after_s is not None:
        data["cancel_after_s"] = cancel_after_s
    path = tmp_path / "t_wp.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, ros2, args, mode="", env_extra=None):
    """Run the runner (no --bench) with the fake ros2 first on PATH.

    Returns (exit code, output, call-log entries, wall time in s).
    """
    bindir, log = ros2
    env = dict(os.environ, FAKE_ROS2_MODE=mode, FAKE_ROS2_LOG=str(log))
    env.pop("FAKE_CLIENT_EXIT", None)
    env |= env_extra or {}
    env["PATH"] = os.pathsep.join([str(bindir), env.get("PATH", "")])
    env["PYTHONPATH"] = str(REPO / "src/eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *map(str, args), "--no-report"]
    cmd += ["--out", str(tmp_path / "out")]
    start = time.monotonic()
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=240)
    elapsed = time.monotonic() - start
    entries = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, entries, elapsed


def launches(entries):
    return [e for e in entries if "argv" in e and e["argv"][:1] == ["launch"]]


def clients(entries):
    return [e for e in entries if "client" in e]


def launch_args(argv):
    return dict(a.split(":=", 1) for a in argv if ":=" in a)


def option(argv, name):
    assert argv.count(name) == 1, argv
    return argv[argv.index(name) + 1]


# ---------------------------------------------------------------- launch and client calls


def test_waypoint_scenario_launches_external_then_runs_client(tmp_path, ros2):
    path = scenario(tmp_path)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    assert "T-wp" in out and "PASS" in out
    (call,) = launches(entries)
    argv = call["argv"]
    assert argv[:3] == ["launch", "cobot_sim", "sim.launch.py"]
    assert "gui:=false" in argv and "controller:=cobot_controller" in argv
    args = launch_args(argv)
    assert args["command_source"] == "external"
    assert Path(args["config_path"]) == short_config(tmp_path)
    assert Path(args["imperfections_path"]) == REALISTIC
    assert args["seed"] == "7"
    assert Path(args["log_path"]).resolve().is_relative_to((tmp_path / "out").resolve())

    (client,) = clients(entries)
    argv = client["client"]
    assert argv[:2] == ["cobot_ros2", "send_waypoints.py"]
    assert Path(argv[2]).is_absolute() and Path(argv[2]) == tmp_path / "wp.yaml"
    assert "--cancel-after" not in argv
    assert client["launch_running"], "client must run while the launch is up"
    # The launch is stopped after the client returned (SIGINT or SIGTERM logged after it).
    order = [e for e in entries if "client" in e or "signal" in e]
    assert ["client" in e for e in order] == [True, False]


def test_client_runs_only_after_standby_on_status(tmp_path, ros2):
    # The fake reports `idle` for the first 3 status messages of the launch, then `standby`.
    path = scenario(tmp_path)
    env = {"FAKE_ROS2_STANDBY_AFTER": "3"}
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path], env_extra=env)
    assert code == 0, out
    echoes = [e for e in entries if "argv" in e and e["argv"][:2] == ["topic", "echo"]]
    assert echoes, "runner must read ~/status with `ros2 topic echo`"
    assert all(STATUS_TOPIC in e["argv"] for e in echoes)
    first_standby = next(k for k, e in enumerate(entries) if "standby" in e.get("status", []))
    first_client = next(k for k, e in enumerate(entries) if "client" in e)
    assert first_standby < first_client
    seen = [m for e in entries if "status" in e for m in e["status"]]
    assert seen.count("idle") >= 3


def test_plain_gazebo_scenario_gets_no_command_source_and_no_client(tmp_path, ros2):
    path = scenario(tmp_path, waypoints=False)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    (call,) = launches(entries)
    assert "command_source" not in launch_args(call["argv"])
    assert not clients(entries)


def test_cancel_after_is_passed_to_the_client(tmp_path, ros2):
    path = scenario(tmp_path, cancel_after_s=3)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    (client,) = clients(entries)
    assert float(option(client["client"], "--cancel-after")) == 3.0


# ---------------------------------------------------------------- verdicts


@pytest.mark.parametrize(
    ("mode", "client_exit", "expected"),
    # Final tip error of the fake: 0 (empty mode) or 10 mm (fail); E2E-02 limit 5 mm.
    # Client exit 0 = goal succeeded, 1 = aborted or rejected.
    [("", "0", 0), ("fail", "0", 1), ("", "1", 1)],
    ids=["pass", "csv-over-threshold", "goal-failed"],
)
def test_waypoint_run_verdict(tmp_path, ros2, mode, client_exit, expected):
    path = scenario(tmp_path)
    env = {"FAKE_CLIENT_EXIT": client_exit}
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path], mode, env)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)
    assert len(launches(entries)) == 1


@pytest.mark.parametrize(
    ("mode", "client_exit", "expected"),
    # Fake CSV (1 ms rows, 0 .. 1.0 s): second half (t >= 0.5 s) in standby, qd = 0,
    # tau = 1 N m (smallest tau_max 12 N m: ratio 1/12 <= 1). moving: qd1 = 0.05 rad/s on the
    # last 50 rows (t >= 0.951 s, inside the last 0.5 s), 0.05 >= 0.01 -> FAIL. overtau:
    # tau1 = 1000 N m on the first row, 1000/87 = 11.5 > 1 -> FAIL. holdend: last row in holding,
    # not standby -> FAIL. Client exit 0: the goal completed instead of being canceled -> FAIL;
    # exit 1: aborted -> FAIL.
    [
        ("", "3", 0),
        ("", "0", 1),
        ("", "1", 1),
        ("moving", "3", 1),
        ("overtau", "3", 1),
        ("holdend", "3", 1),
    ],
    ids=["canceled", "completed", "aborted", "still-moving", "over-torque", "not-standby"],
)
def test_cancel_verdict(tmp_path, ros2, mode, client_exit, expected):
    path = scenario(tmp_path, cancel_after_s=3)
    env = {"FAKE_CLIENT_EXIT": client_exit}
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path], mode, env)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)
    assert len(launches(entries)) == 1


def test_cancel_without_waypoints_fails(tmp_path, ros2):
    path = scenario(tmp_path, waypoints=False, cancel_after_s=3)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 1, out
    assert "FAIL" in out
    assert not clients(entries)


def test_launch_dying_before_standby_fails_promptly(tmp_path, ros2):
    path = scenario(tmp_path)
    code, out, entries, elapsed = run_e2e(tmp_path, ros2, ["--scenario", path], "crash")
    assert code == 1, out
    assert "FAIL" in out
    assert not clients(entries), "client run although the launch never reached standby"
    assert elapsed < 30.0, f"runner took {elapsed:.1f} s to notice the dead launch"


# ---------------------------------------------------------------- E2E-11 scenario files

E2E_10_WAYPOINTS = REPO / "tests/e2e/waypoints/e2e_10.yaml"


def load_scenario(name):
    return yaml.safe_load((REPO / "tests/e2e/scenarios" / name).read_text(encoding="utf-8"))


def test_e2e_11_scenario_follows_contract():
    s = load_scenario("e2e_11_gazebo_waypoints.yaml")
    assert s["id"] == "E2E-11"
    assert s["runner"] == "gazebo" and s["kind"] == "run" and s["tags"] == ["gazebo"]
    assert (REPO / s["waypoints"]).resolve() == E2E_10_WAYPOINTS.resolve()
    assert E2E_10_WAYPOINTS.is_file()
    assert (REPO / s["imperfections"]).resolve() == REALISTIC.resolve()
    assert s["seed"] == 7
    assert "cancel_after_s" not in s
    # Thresholds as E2E-10: the E2E-02 values (plus E2E-10's optional extra keys).
    for key, value in G2.items():
        assert s["thresholds"][key] == value


def test_e2e_11b_scenario_follows_contract():
    s = load_scenario("e2e_11b_gazebo_cancel.yaml")
    assert s["id"] == "E2E-11b"
    assert s["runner"] == "gazebo" and s["kind"] == "run" and s["tags"] == ["gazebo"]
    assert (REPO / s["waypoints"]).resolve() == E2E_10_WAYPOINTS.resolve()
    assert (REPO / s["imperfections"]).resolve() == REALISTIC.resolve()
    assert s["seed"] == 7
    assert float(s["cancel_after_s"]) == 3.0
    assert (REPO / s["config"]).is_file()
