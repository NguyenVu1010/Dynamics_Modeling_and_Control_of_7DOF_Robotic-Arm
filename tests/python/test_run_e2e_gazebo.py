"""Gazebo branch of tools/run_e2e.py against a fake `ros2` on PATH (fake_ros2.py, no Gazebo).

The runner launches `ros2 launch cobot_sim sim.launch.py gui:=false controller:=cobot_controller
log_path:=<csv under --out>`, waits duration_s + hold_s + 5 s (trajectory keys of the scenario's
config), stops the launch, then judges the CSV at log_path with the scenario thresholds.
Gazebo scenarios need no --bench (tools/ci/gazebo_ci.sh passes none).
"""

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
IDEAL = REPO / "config/imperfections/ideal.yaml"
# default.yaml without friction compensation (the ideal plant has no friction).
IDEAL_WORLD = REPO / "tests/e2e/configs/ideal_world.yaml"
G3 = {"final_pos_err_m": 0.002, "final_rot_err_deg": 1.0, "rms_joint_err_rad": 0.01}
DURATION_S = 0.5
HOLD_S = 0.5
MARGIN_S = 5.0
# Shortest wait the runner may take before stopping the launch: 0.5 + 0.5 + 5 = 6.0 s.
MIN_WAIT_S = DURATION_S + HOLD_S + MARGIN_S


@pytest.fixture
def ros2(tmp_path):
    """Directory holding an executable fake `ros2`; returns (bin dir, JSON-lines call log)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "ros2"
    source = (Path(__file__).parent / "fake_ros2.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return bindir, tmp_path / "ros2_calls.jsonl"


@pytest.fixture
def ros2_external(tmp_path):
    """As `ros2`, with fake_ros2_external.py: also answers `topic echo` on the controller status
    and `run cobot_ros2 send_waypoints.py` (waypoint scenarios, feature waypoint-ros)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "ros2"
    source = (Path(__file__).parent / "fake_ros2_external.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return bindir, tmp_path / "ros2_calls.jsonl"


def short_config(tmp_path):
    """default.yaml with duration_s 0.5 and hold_s 0.5, paths made absolute."""
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    cobot = data["cobot"]
    cobot["urdf_path"] = str((CONFIG.parent / cobot["urdf_path"]).resolve())
    cobot["log"]["output_dir"] = str(tmp_path / "results")
    cobot["trajectory"]["duration_s"] = DURATION_S
    cobot["trajectory"]["hold_s"] = HOLD_S
    path = tmp_path / "short.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def scenario(tmp_path, thresholds):
    data = {"id": "T-gazebo", "tags": ["gazebo"], "runner": "gazebo", "kind": "run"}
    data |= {"config": str(short_config(tmp_path)), "imperfections": str(IDEAL), "seed": 1}
    data["thresholds"] = thresholds
    path = tmp_path / "t_gazebo.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, ros2, args, mode="", env_extra=None):
    """Run the runner (no --bench) with the fake ros2 first on PATH.

    Returns (exit code, output, call-log entries, wall time in s).
    """
    bindir, log = ros2
    env = dict(os.environ, FAKE_ROS2_MODE=mode, FAKE_ROS2_LOG=str(log), **(env_extra or {}))
    env["PATH"] = os.pathsep.join([str(bindir), env.get("PATH", "")])
    env["PYTHONPATH"] = str(REPO / "src/eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *map(str, args), "--no-report"]
    cmd += ["--out", str(tmp_path / "out")]
    start = time.monotonic()
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=180)
    elapsed = time.monotonic() - start
    entries = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, entries, elapsed


def launches(entries):
    return [e for e in entries if "argv" in e and e["argv"][:1] == ["launch"]]


def launch_arg(argv, name):
    values = [a.split(":=", 1)[1] for a in argv if a.startswith(f"{name}:=")]
    assert len(values) == 1, argv
    return values[0]


def is_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_gazebo_scenario_file_follows_contract():
    path = REPO / "tests/e2e/scenarios/e2e_07_gazebo.yaml"
    s = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert s["id"] == "E2E-07"
    assert s["runner"] == "gazebo" and s["kind"] == "run" and s["tags"] == ["gazebo"]
    assert (REPO / s["config"]).resolve() == IDEAL_WORLD.resolve()
    assert (REPO / s["imperfections"]).resolve() == IDEAL.resolve()
    assert isinstance(s["seed"], int)
    assert s["thresholds"] == G3


def test_gazebo_launch_command_csv_and_stop(tmp_path, ros2):
    code, out, entries, elapsed = run_e2e(tmp_path, ros2, ["--scenario", scenario(tmp_path, G3)])
    assert code == 0, out
    assert "T-gazebo" in out and "PASS" in out
    (call,) = launches(entries)
    argv = call["argv"]
    assert argv[:3] == ["launch", "cobot_sim", "sim.launch.py"]
    assert "gui:=false" in argv and "controller:=cobot_controller" in argv
    csv = Path(launch_arg(argv, "log_path"))
    assert csv.suffix == ".csv"
    assert csv.resolve().is_relative_to((tmp_path / "out").resolve())
    # Located and complete: header + 1001 rows (t = 0 .. 1.0 s at 1 ms), the second half
    # written only after the stop signal.
    assert len(csv.read_text(encoding="utf-8").splitlines()) == 1 + 1001
    # Stopped gracefully (the controller flushes its CSV on shutdown), after the full wait.
    assert [e["signal"] for e in entries if "signal" in e] in (
        [signal.SIGINT],
        [signal.SIGTERM],
    )
    assert elapsed >= MIN_WAIT_S, f"runner stopped the launch after {elapsed:.2f} s"
    deadline = time.monotonic() + 5.0
    while is_alive(call["pid"]) and time.monotonic() < deadline:
        time.sleep(0.1)
    assert not is_alive(call["pid"]), "launch process left running"


@pytest.mark.parametrize(
    ("mode", "limit_m", "expected"),
    # Final tip error of the fake: 10 mm (fail), 4 mm (pos4mm). 10 mm > 2 mm fails, 4 mm <= 5 mm
    # passes, 4 mm > 2 mm fails. nocsv: no CSV at log_path; crash: launch exits 3 at once.
    [
        ("fail", 0.002, 1),
        ("pos4mm", 0.005, 0),
        ("pos4mm", 0.002, 1),
        ("nocsv", 0.002, 1),
        ("crash", 0.002, 1),
    ],
)
def test_gazebo_verdict_uses_scenario_thresholds(tmp_path, ros2, mode, limit_m, expected):
    path = scenario(tmp_path, G3 | {"final_pos_err_m": limit_m})
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path], mode)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)
    assert len(launches(entries)) == 1


def test_all_tag_gazebo_runs_e2e_07_without_bench(tmp_path, ros2_external):
    # Real scenario: ideal_world.yaml has duration_s 10 + hold_s 2, so the fake writes rows up to
    # 12 s and the runner waits at least 10 + 2 + 5 = 17 s. Bench scenarios are not tagged
    # gazebo, so no --bench is needed.
    env = {"FAKE_ROS2_T_END_S": "12.0"}
    code, out, entries, elapsed = run_e2e(
        tmp_path, ros2_external, ["--all", "--tag", "gazebo"], env_extra=env
    )
    assert code == 0, out
    assert "E2E-07" in out and "PASS" in out
    # E2E-07 (ideal), E2E-09 (realistic), E2E-11 and E2E-11b (waypoints through the action,
    # feature waypoint-ros) carry the gazebo tag.
    calls = launches(entries)
    assert len(calls) == 4
    assert all("controller:=cobot_controller" in c["argv"] for c in calls)
    assert elapsed >= 17.0
