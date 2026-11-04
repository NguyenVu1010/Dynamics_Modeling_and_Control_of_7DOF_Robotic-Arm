"""tools/run_e2e.py gazebo runner and the black-box log (feature blackbox-log), against a fake
`ros2` on PATH (fake_ros2.py / fake_ros2_external.py, no Gazebo).

Contract pinned here:
- every gazebo launch gets `log_stream_ms:=200` (the controller appends its log to log_path every
  200 ms, so wait_run and drive_waypoints keep tailing run.csv);
- before the launch is stopped, the runner calls
  `ros2 service call /cobot_controller/dump_log std_srvs/srv/Trigger` once, while the launch is
  up; a failing or hanging call (timeout 10 s) is logged, not fatal;
- after the launch is stopped, when run.csv is missing or shorter than the run needs, the runner
  runs `ros2 run cobot_ros2 cobot_logdump cobot_controller --out <csv>` and judges that file;
  a complete run.csv needs no cobot_logdump.
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
IDEAL = REPO / "config/imperfections/ideal.yaml"
REALISTIC = REPO / "config/imperfections/realistic.yaml"
G3 = {"final_pos_err_m": 0.002, "final_rot_err_deg": 1.0, "rms_joint_err_rad": 0.01}
G2 = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}
DUMP_LOG = ["/cobot_controller/dump_log", "std_srvs/srv/Trigger"]
# Fake run: t = 0 .. 1.0 s at 1 ms = 1001 rows, plus the header.
FULL_LINES = 1 + 1001
# Shortest wait before the stop: duration_s 0.5 + hold_s 0.5 + 5 s margin.
MIN_WAIT_S = 6.0


def fake_ros2(tmp_path, script):
    bindir = tmp_path / "bin"
    bindir.mkdir()
    exe = bindir / "ros2"
    source = (Path(__file__).parent / script).read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return bindir, tmp_path / "ros2_calls.jsonl"


@pytest.fixture
def ros2(tmp_path):
    """(bin dir, JSON-lines call log) of fake_ros2.py as `ros2`."""
    return fake_ros2(tmp_path, "fake_ros2.py")


@pytest.fixture
def ros2_external(tmp_path):
    """(bin dir, JSON-lines call log) of fake_ros2_external.py as `ros2`."""
    return fake_ros2(tmp_path, "fake_ros2_external.py")


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


def plain_scenario(tmp_path, thresholds=None):
    data = {"id": "T-dump", "tags": ["gazebo"], "runner": "gazebo", "kind": "run"}
    data |= {"config": str(short_config(tmp_path)), "imperfections": str(IDEAL), "seed": 1}
    data["thresholds"] = thresholds or G3
    path = tmp_path / "t_dump.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def waypoint_scenario(tmp_path):
    wp = tmp_path / "wp.yaml"
    first = {"position_m": [-0.5, -0.6, 0.9], "rpy_deg": [0, 0, 90]}
    second = {"position_m": [-0.3, -0.6, 0.7], "rpy_deg": [0, 0, 90]}
    wp_data = {
        "frame": "base_link",
        "waypoints": [
            first | {"duration_s": 0, "stop": False},
            second | {"duration_s": 4.0, "stop": True},
        ],
    }
    wp.write_text(yaml.safe_dump(wp_data), encoding="utf-8")
    data = {"id": "T-wp-dump", "tags": ["gazebo"], "runner": "gazebo", "kind": "run"}
    data |= {"config": str(short_config(tmp_path)), "imperfections": str(REALISTIC), "seed": 7}
    data |= {"thresholds": G2, "waypoints": str(wp)}
    path = tmp_path / "t_wp_dump.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, ros2, scenario, mode=""):
    """Runner on one scenario (no --bench) with the fake ros2 first on PATH.

    Returns (exit code, output, call-log entries, wall time in s).
    """
    bindir, log = ros2
    env = dict(os.environ, FAKE_ROS2_MODE=mode, FAKE_ROS2_LOG=str(log))
    env["PATH"] = os.pathsep.join([str(bindir), env.get("PATH", "")])
    env["PYTHONPATH"] = str(REPO / "src/eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), "--scenario", str(scenario)]
    cmd += ["--no-report", "--out", str(tmp_path / "out")]
    start = time.monotonic()
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=240)
    elapsed = time.monotonic() - start
    entries = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, entries, elapsed


def launches(entries):
    return [e for e in entries if "argv" in e and e["argv"][:1] == ["launch"]]


def launch_arg(argv, name):
    values = [a.split(":=", 1)[1] for a in argv if a.startswith(f"{name}:=")]
    assert len(values) == 1, argv
    return values[0]


def index_of(entries, key):
    """Positions of the call-log entries carrying `key`."""
    return [k for k, e in enumerate(entries) if key in e]


def logdump_out(entry):
    argv = entry["logdump"]
    assert argv[:3] == ["cobot_ros2", "cobot_logdump", "cobot_controller"], argv
    assert argv.count("--out") == 1, argv
    return Path(argv[argv.index("--out") + 1])


# ---------------------------------------------------------------- log_stream_ms and ~/dump_log


def test_launch_streams_the_log_every_200_ms(tmp_path, ros2):
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path))
    assert code == 0, out
    (call,) = launches(entries)
    assert launch_arg(call["argv"], "log_stream_ms") == "200"


def test_dump_log_is_called_once_before_the_stop_and_complete_csv_needs_no_logdump(tmp_path, ros2):
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path))
    assert code == 0, out
    assert "PASS" in out
    dumps = index_of(entries, "dump_log")
    assert len(dumps) == 1, entries
    dump = entries[dumps[0]]
    assert dump["dump_log"][:2] == DUMP_LOG
    assert dump["launch_running"], "~/dump_log must be called while the launch is up"
    signals = index_of(entries, "signal")
    assert len(signals) == 1
    assert dumps[0] < signals[0], "~/dump_log after the launch was stopped"
    # run.csv is complete (header + 1001 rows): no fallback.
    assert not index_of(entries, "logdump"), entries


def test_waypoint_scenario_streams_and_dumps_after_the_client(tmp_path, ros2_external):
    code, out, entries, _ = run_e2e(tmp_path, ros2_external, waypoint_scenario(tmp_path))
    assert code == 0, out
    (call,) = launches(entries)
    assert launch_arg(call["argv"], "log_stream_ms") == "200"
    assert launch_arg(call["argv"], "command_source") == "external"
    clients = index_of(entries, "client")
    dumps = index_of(entries, "dump_log")
    signals = index_of(entries, "signal")
    assert len(clients) == 1 and len(dumps) == 1 and len(signals) == 1, entries
    assert clients[0] < dumps[0] < signals[0]
    assert entries[dumps[0]]["dump_log"][:2] == DUMP_LOG
    assert entries[dumps[0]]["launch_running"]
    assert not index_of(entries, "logdump"), entries


def test_dump_log_failure_is_not_fatal(tmp_path, ros2):
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path), "dumpfail")
    assert code == 0, out
    assert "PASS" in out
    assert len(index_of(entries, "dump_log")) == 1


def test_hanging_dump_log_times_out_after_10_s(tmp_path, ros2):
    # The fake answers after 60 s; with the 10 s timeout the run takes about 6 + 10 s.
    code, out, entries, elapsed = run_e2e(tmp_path, ros2, plain_scenario(tmp_path), "dumphang")
    assert code == 0, out
    assert "PASS" in out
    assert len(index_of(entries, "dump_log")) == 1
    assert elapsed < MIN_WAIT_S + 10.0 + 20.0, f"runner waited {elapsed:.1f} s on ~/dump_log"


# ---------------------------------------------------------------- cobot_logdump fallback


def expect_logdump_after_stop(entries, tmp_path):
    """The one cobot_logdump call ran after the launch was stopped; returns its --out file."""
    dumps = index_of(entries, "logdump")
    assert len(dumps) == 1, entries
    signals = index_of(entries, "signal")
    assert len(signals) == 1 and signals[0] < dumps[0], "cobot_logdump before the launch stopped"
    csv = logdump_out(entries[dumps[0]])
    if not csv.is_absolute():
        csv = tmp_path / csv  # the runner's working directory
    return csv


def test_short_csv_falls_back_to_cobot_logdump(tmp_path, ros2):
    # The fake controller dies before writing the second half: run.csv covers 0 .. 0.499 s of
    # the 1.0 s run (first half, 50 mm transient error, which alone would fail G3). The region
    # dumped by cobot_logdump holds all 1001 rows ending at 0 mm: PASS on that file.
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path), "shortcsv")
    assert code == 0, out
    assert "PASS" in out
    csv = expect_logdump_after_stop(entries, tmp_path)
    assert csv.resolve().is_relative_to((tmp_path / "out").resolve()), csv
    assert len(csv.read_text(encoding="utf-8").splitlines()) == FULL_LINES


def test_missing_csv_falls_back_to_cobot_logdump(tmp_path, ros2):
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path), "csvlost")
    assert code == 0, out
    assert "PASS" in out
    csv = expect_logdump_after_stop(entries, tmp_path)
    assert len(csv.read_text(encoding="utf-8").splitlines()) == FULL_LINES


def test_waypoint_scenario_falls_back_to_cobot_logdump(tmp_path, ros2_external):
    code, out, entries, _ = run_e2e(tmp_path, ros2_external, waypoint_scenario(tmp_path), "csvlost")
    assert code == 0, out
    assert "PASS" in out
    csv = expect_logdump_after_stop(entries, tmp_path)
    assert len(csv.read_text(encoding="utf-8").splitlines()) == FULL_LINES


@pytest.mark.parametrize(
    "mode",
    # csvlost,fail: the dumped file ends at 10 mm > 2 mm (G3) -> the verdict is taken on it.
    # csvlost,noregion and nocsv: cobot_logdump exits 1 (no region) -> no CSV to judge.
    ["csvlost,fail", "csvlost,noregion", "nocsv"],
)
def test_fallback_verdict(tmp_path, ros2, mode):
    code, out, entries, _ = run_e2e(tmp_path, ros2, plain_scenario(tmp_path), mode)
    assert code == 1, out
    assert "FAIL" in out
    expect_logdump_after_stop(entries, tmp_path)
