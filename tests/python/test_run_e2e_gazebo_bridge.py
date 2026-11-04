"""tools/run_e2e.py, Gazebo scenarios driven through the studio bridge (feature controller-bridge,
E2E-13), against a fake `ros2` on PATH (fake_ros2_bridge.py: no Gazebo, no rclpy).

Contract pinned here:
- a gazebo scenario with `bridge_steps: [<cli step>, ...]` launches as today (config_path,
  imperfections_path, seed, log_path) plus `studio:=true`, waits for `standby` on
  /cobot_controller/status, then runs `ros2 run cobot_ros2 cobot_bridge_cli cobot_controller
  <steps>` while the launch is up (output in <out>/bridge.log), and stops the launch after it;
  `<out>` in a step is the scenario's output directory;
- with `panel_waypoints: FILE`, the runner runs `ros2 run cobot_ros2 send_waypoints.py <absolute
  FILE>` during the CLI's pause (the bridge client still running); only exit 1 with "studio is in
  control" in its output passes;
- PASS needs CLI exit 0, that panel refusal, a Run CSV with a row of every `expect.cmd` value, no
  `holding` row and a last row in standby, and the CSV of the last `export:` step within the
  scenario thresholds (judge_run); `bridge_steps` together with `waypoints` is a FAIL;
- tests/e2e/scenarios/e2e_13_gazebo_bridge.yaml follows the brief.
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
E2E_13_WAYPOINTS = REPO / "tests/e2e/waypoints/e2e_13.yaml"
# E2E-02 thresholds.
G2 = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}
CMDS = ["jog_joint", "move_joint", "move_tcp", "waypoints"]
SID = "T-br"


@pytest.fixture
def ros2(tmp_path):
    """Directory holding an executable fake `ros2`; returns (bin dir, JSON-lines call log)."""
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "ros2"
    source = (Path(__file__).parent / "fake_ros2_bridge.py").read_text(encoding="utf-8")
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


def steps_of(pause_s=2):
    return [
        "claim",
        f"pause:{pause_s}",
        "jog-joint:1:+:0.5",
        "move-joint:0,30,0,-90,0,60,0",
        "move-tcp:0.160012653,-0.806959296,0.389122496,0,-90,90",
        f"waypoints:{E2E_13_WAYPOINTS}:2",
        "export:<out>/segment.csv",
        "release",
    ]


def scenario(tmp_path, *, panel=True, waypoints=False, steps=None, expect=None):
    data = {"id": SID, "tags": ["gazebo"], "runner": "gazebo", "kind": "run"}
    data |= {"config": str(short_config(tmp_path)), "imperfections": str(REALISTIC), "seed": 7}
    data["thresholds"] = G2
    data["bridge_steps"] = steps if steps is not None else steps_of()
    data["expect"] = {"cmd": expect if expect is not None else CMDS}
    if panel:
        data["panel_waypoints"] = str(E2E_13_WAYPOINTS)
    if waypoints:
        data["waypoints"] = str(E2E_13_WAYPOINTS)
    path = tmp_path / "t_br.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, ros2, args, mode="", env_extra=None):
    """Run the runner (no --bench) with the fake ros2 first on PATH.

    Returns (exit code, output, call-log entries, wall time in s).
    """
    bindir, log = ros2
    env = dict(os.environ, FAKE_ROS2_MODE=mode, FAKE_ROS2_LOG=str(log))
    for name in ("FAKE_BRIDGE_EXIT", "FAKE_PANEL_EXIT", "FAKE_CLIENT_EXIT"):
        env.pop(name, None)
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


def bridges(entries):
    return [e for e in entries if "bridge" in e]


def panels(entries):
    return [e for e in entries if "panel" in e]


def launch_args(argv):
    return dict(a.split(":=", 1) for a in argv if ":=" in a)


# ---------------------------------------------------------------- launch, CLI and panel calls


def test_bridge_scenario_launches_studio_and_runs_the_cli(tmp_path, ros2):
    path = scenario(tmp_path)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    assert SID in out and "PASS" in out

    (call,) = launches(entries)
    argv = call["argv"]
    assert argv[:3] == ["launch", "cobot_sim", "sim.launch.py"]
    assert "gui:=false" in argv and "controller:=cobot_controller" in argv
    args = launch_args(argv)
    assert args["studio"] == "true"
    assert Path(args["config_path"]) == short_config(tmp_path)
    assert Path(args["imperfections_path"]) == REALISTIC
    assert args["seed"] == "7"
    assert Path(args["log_path"]).resolve().is_relative_to((tmp_path / "out").resolve())

    (cli,) = bridges(entries)
    argv = cli["bridge"]
    assert argv[:3] == ["cobot_ros2", "cobot_bridge_cli", "cobot_controller"]
    out_dir = (tmp_path / "out" / SID).resolve()
    want = [s.replace("<out>", str(out_dir)) for s in steps_of()]
    got = argv[-len(want) :]
    assert got[:-2] == want[:-2]
    assert got[-1] == "release"
    export = got[-2]
    assert export.startswith("export:")
    assert Path(export.split(":", 1)[1]).resolve() == out_dir / "segment.csv"
    assert cli["launch_running"], "the CLI must run while the launch is up"
    log = (out_dir / "bridge.log").read_text(encoding="utf-8")
    assert "1 claim: ok" in log

    # Standby first, then the CLI, then the stop of the launch.
    first_standby = next(k for k, e in enumerate(entries) if "standby" in e.get("status", []))
    first_cli = next(k for k, e in enumerate(entries) if "bridge" in e)
    assert first_standby < first_cli
    order = [e for e in entries if "bridge" in e or "signal" in e]
    assert ["bridge" in e for e in order] == [True, False]


def test_panel_client_runs_during_the_pause_and_is_refused(tmp_path, ros2):
    path = scenario(tmp_path)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    (call,) = panels(entries)
    argv = call["panel"]
    assert argv[:2] == ["cobot_ros2", "send_waypoints.py"]
    assert Path(argv[2]).is_absolute() and Path(argv[2]) == E2E_13_WAYPOINTS
    assert call["bridge_running"], "the panel client must run while the CLI holds authority"
    assert call["launch_running"]


def test_without_panel_waypoints_no_panel_client_runs(tmp_path, ros2):
    path = scenario(tmp_path, panel=False)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 0, out
    assert not panels(entries)
    assert len(bridges(entries)) == 1


def test_bridge_steps_with_waypoints_fail(tmp_path, ros2):
    path = scenario(tmp_path, waypoints=True)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path])
    assert code == 1, out
    assert "FAIL" in out
    assert not bridges(entries)


# ---------------------------------------------------------------- verdicts


@pytest.mark.parametrize(
    ("mode", "env", "expected"),
    # Fake Run CSV: every cmd of CMDS, no holding row, last row standby; segment CSV: tip on its
    # reference (0 mm, E2E-02 limit 5 mm). segfail: 10 mm off -> FAIL. nomovetcp: no move_tcp
    # row -> FAIL. holding: one holding row -> FAIL. tracklast: last row tracking -> FAIL.
    # FAKE_BRIDGE_EXIT 1: a step failed -> FAIL. FAKE_PANEL_EXIT 0: the panel goal was accepted
    # -> FAIL. panelquiet: exit 1 without "studio is in control" -> FAIL.
    [
        ("", {}, 0),
        ("segfail", {}, 1),
        ("nomovetcp", {}, 1),
        ("holding", {}, 1),
        ("tracklast", {}, 1),
        ("", {"FAKE_BRIDGE_EXIT": "1"}, 1),
        ("", {"FAKE_PANEL_EXIT": "0"}, 1),
        ("panelquiet", {}, 1),
    ],
    ids=[
        "pass",
        "segment-over-threshold",
        "cmd-missing",
        "holding-row",
        "last-not-standby",
        "cli-failed",
        "panel-accepted",
        "panel-without-reason",
    ],
)
def test_bridge_verdict(tmp_path, ros2, mode, env, expected):
    path = scenario(tmp_path)
    code, out, entries, _ = run_e2e(tmp_path, ros2, ["--scenario", path], mode, env)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)
    assert len(launches(entries)) == 1


def test_launch_dying_before_standby_fails_without_running_the_cli(tmp_path, ros2):
    path = scenario(tmp_path)
    code, out, entries, elapsed = run_e2e(tmp_path, ros2, ["--scenario", path], "crash")
    assert code == 1, out
    assert "FAIL" in out
    assert not bridges(entries)
    assert elapsed < 30.0, f"runner took {elapsed:.1f} s to notice the dead launch"


# ---------------------------------------------------------------- E2E-13 scenario file


def load_scenario(name):
    return yaml.safe_load((REPO / "tests/e2e/scenarios" / name).read_text(encoding="utf-8"))


def test_e2e_13_scenario_follows_the_brief():
    s = load_scenario("e2e_13_gazebo_bridge.yaml")
    assert s["id"] == "E2E-13"
    assert s["runner"] == "gazebo" and s["kind"] == "run" and s["tags"] == ["gazebo"]
    assert (REPO / s["config"]).resolve() == CONFIG.resolve()
    assert (REPO / s["imperfections"]).resolve() == REALISTIC.resolve()
    assert s["seed"] == 7
    assert "waypoints" not in s
    assert (REPO / s["panel_waypoints"]).resolve() == E2E_13_WAYPOINTS.resolve()
    for key, value in G2.items():
        assert s["thresholds"][key] == value
    assert s["expect"]["cmd"] == CMDS
    steps = s["bridge_steps"]
    home_deg = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["cobot"]["home_q_deg"]
    assert steps[0] == "claim"
    assert steps[1] == "pause:20"
    assert steps[2] == "jog-joint:1:+:0.5"
    assert [float(v) for v in steps[3].removeprefix("move-joint:").split(",")] == home_deg
    assert steps[4].startswith("move-tcp:") and len(steps[4].split(":")[1].split(",")) == 6
    assert steps[5] == "waypoints:tests/e2e/waypoints/e2e_13.yaml:2"
    assert steps[6] == "export:<out>/segment.csv"
    assert steps[7] == "release"
    assert len(steps) == 8


def test_e2e_13_waypoint_file_has_two_points():
    data = yaml.safe_load(E2E_13_WAYPOINTS.read_text(encoding="utf-8"))
    assert data["frame"] == "base_link"
    assert len(data["waypoints"]) == 2
    assert data["waypoints"][-1]["stop"] is True
