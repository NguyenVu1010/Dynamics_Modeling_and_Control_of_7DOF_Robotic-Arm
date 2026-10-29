"""E2E-10 and the `waypoints:` scenario key of tools/run_e2e.py (feature bench-waypoints).

A bench scenario with `waypoints: <path>` (relative to the repo root, or absolute) runs
`cobot_bench run ... --waypoints <path>` for each of its runs; without the key no --waypoints is
passed. `thresholds.stop_pos_err_m` is optional and judged from the run's meta file
(cobot_eval.metrics.stop_pos_err_m). Fake bench: fake_bench.py.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
IDEAL = REPO / "config/imperfections/ideal.yaml"
REALISTIC = REPO / "config/imperfections/realistic.yaml"
SCENARIOS = REPO / "tests/e2e/scenarios"
E2E_10 = SCENARIOS / "e2e_10_waypoints.yaml"
E2E_10_WAYPOINTS = REPO / "tests/e2e/waypoints/e2e_10.yaml"
G3_REAL = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}
BENCH = Path(os.environ.get("COBOT_BENCH", REPO / "build/bench/cobot_bench"))


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, kind="run", **fields):
    data = {"id": f"T-{kind}", "tags": ["pr"], "runner": "bench", "kind": kind}
    data |= {"config": str(CONFIG), "imperfections": str(IDEAL), "seed": 1, **fields}
    path = tmp_path / f"{data['id']}.yaml"
    path.write_bytes(yaml.safe_dump(data).encode())
    return path


def run_e2e(tmp_path, bench, args, mode=""):
    """Runner from an unrelated cwd, --no-report; returns (exit code, output, bench argv list)."""
    exe, log = bench
    env = dict(os.environ, FAKE_BENCH_MODE=mode, FAKE_BENCH_LOG=str(log))
    env["PYTHONPATH"] = str(REPO / "eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *map(str, args), "--no-report"]
    cmd += ["--bench", str(exe), "--out", str(tmp_path / "out")]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=300)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, calls


def flag(argv, name):
    return argv[argv.index(name) + 1]


def test_e2e_10_scenario_follows_contract():
    s = load(E2E_10)
    assert s["id"] == "E2E-10"
    assert s["tags"] == ["pr", "arm64"]
    assert s["runner"] == "bench" and s["kind"] == "run"
    assert (REPO / s["config"]).resolve() == CONFIG.resolve()
    assert (REPO / s["imperfections"]).resolve() == REALISTIC.resolve()
    assert type(s["seed"]) is int and s["seed"] == 7
    assert (REPO / s["waypoints"]).resolve() == E2E_10_WAYPOINTS.resolve()
    assert s["thresholds"] == G3_REAL | {"stop_pos_err_m": 0.005}


def test_e2e_10_thresholds_are_e2e_02_plus_stop():
    th = dict(load(E2E_10)["thresholds"])
    assert th.pop("stop_pos_err_m") == 0.005
    assert th == load(SCENARIOS / "e2e_02_realistic.yaml")["thresholds"]


def test_e2e_10_waypoint_file_follows_contract():
    # Four points, the second one stops (and the last), auto (0) and user durations mixed.
    w = load(E2E_10_WAYPOINTS)
    assert w["frame"] == "base_link"
    points = w["waypoints"]
    assert len(points) == 4
    assert [bool(p.get("stop", False)) for p in points] == [False, True, False, True]
    durations = [float(p.get("duration_s", 0.0)) for p in points]
    assert any(d == 0.0 for d in durations) and any(d > 0.0 for d in durations)
    for p in points:
        assert len(p["position_m"]) == 3 and len(p["rpy_deg"]) == 3


@pytest.mark.parametrize("relative", [True, False])
def test_waypoints_are_passed_to_the_bench(tmp_path, bench, relative):
    # A relative path resolves against the repo root (the runner runs from tmp_path here).
    value = "tests/e2e/waypoints/e2e_10.yaml" if relative else str(E2E_10_WAYPOINTS)
    path = scenario(tmp_path, waypoints=value, thresholds=G3_REAL)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    (argv,) = calls
    assert argv[0] == "run"
    assert Path(flag(argv, "--waypoints")).resolve() == E2E_10_WAYPOINTS.resolve()


def test_determinism_passes_waypoints_to_both_runs(tmp_path, bench):
    path = scenario(tmp_path, kind="determinism", waypoints=str(E2E_10_WAYPOINTS))
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert [c[0] for c in calls] == ["run", "run"]
    for argv in calls:
        assert Path(flag(argv, "--waypoints")).resolve() == E2E_10_WAYPOINTS.resolve()


def test_no_waypoints_key_passes_no_flag(tmp_path, bench):
    path = scenario(tmp_path, thresholds=G3_REAL)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    (argv,) = calls
    assert "--waypoints" not in argv


@pytest.mark.parametrize(
    ("mode", "expected"),
    # Fake: stop waypoints at the tip (error 0) or 6 mm off (stop6mm); 6 mm > 5 mm fails.
    # pos4mm: final and stop errors 4 mm <= 5 mm pass; fail: 10 mm fails both.
    [("", 0), ("stop6mm", 1), ("pos4mm", 0), ("fail", 1)],
)
def test_stop_pos_err_threshold_is_judged(tmp_path, bench, mode, expected):
    th = G3_REAL | {"stop_pos_err_m": 0.005}
    path = scenario(tmp_path, waypoints=str(E2E_10_WAYPOINTS), thresholds=th)
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], mode)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)
    if mode == "stop6mm":
        assert "stop_pos_err_m" in out


def test_e2e_10_scenario_runs_with_the_fake_bench(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", E2E_10])
    assert code == 0, out
    assert "E2E-10" in out and "PASS" in out
    (argv,) = calls
    assert flag(argv, "--seed") == "7"
    assert Path(flag(argv, "--imperfections")).resolve() == REALISTIC.resolve()
    assert Path(flag(argv, "--waypoints")).resolve() == E2E_10_WAYPOINTS.resolve()


@pytest.mark.skipif(not BENCH.is_file(), reason=f"cobot_bench not built ({BENCH})")
def test_e2e_10_passes_with_the_real_bench(tmp_path):
    # The real check of E2E-10 (set COBOT_BENCH to the built binary): every waypoint validates
    # and the realistic run meets the thresholds.
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), "--scenario", str(E2E_10)]
    cmd += ["--bench", str(BENCH), "--out", str(tmp_path / "out"), "--no-report"]
    env = dict(os.environ, PYTHONPATH=str(REPO / "eval"))
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "PASS" in proc.stdout
