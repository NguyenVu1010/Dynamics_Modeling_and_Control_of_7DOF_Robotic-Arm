"""tools/run_e2e.py against a fake bench (fake_bench.py) and the real scenario files."""

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
IDEAL_WORLD = REPO / "tests/e2e/configs/ideal_world.yaml"
G3 = {"final_pos_err_m": 0.002, "final_rot_err_deg": 1.0, "rms_joint_err_rad": 0.01}
G3_REAL = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}
PR = ["pr", "arm64"]


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, kind="run", **fields):
    """Scenario file with absolute paths (valid whatever base the runner resolves against)."""
    data = {"id": f"T-{kind}", "tags": ["pr"], "runner": "bench", "kind": kind}
    data |= {"config": str(CONFIG), "imperfections": str(IDEAL), "seed": 1, **fields}
    path = tmp_path / f"{data['id']}.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, bench, args, mode="", report=False, pythonpath=()):
    """Run the runner from an unrelated cwd; returns (exit code, output, bench argv list)."""
    exe, log = bench
    env = dict(os.environ, FAKE_BENCH_MODE=mode, FAKE_BENCH_LOG=str(log))
    env["PYTHONPATH"] = os.pathsep.join([*map(str, pythonpath), str(REPO / "eval")])
    args = [str(a) for a in args] + ([] if report else ["--no-report"])
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *args, "--bench", str(exe)]
    cmd += ["--out", str(tmp_path / "out")]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=300)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, calls


def flag(argv, name):
    return argv[argv.index(name) + 1]


@pytest.mark.parametrize(
    ("name", "kind", "tags", "extra"),
    [
        ("e2e_01_ideal", "run", PR, {"seed": 1, "thresholds": G3}),
        ("e2e_02_realistic", "run", PR, {"seed": 7, "thresholds": G3_REAL}),
        ("e2e_03_determinism", "determinism", PR, {}),
        ("e2e_04_safety_limit", "safety", PR, {"expect": {"mode_reached": "holding"}}),
        ("pf_01_timing", "timing", ["pr"], {"thresholds": {"p95_step_us": 1000}}),
    ],
)
def test_scenario_files_follow_contract(name, kind, tags, extra):
    s = yaml.safe_load((REPO / f"tests/e2e/scenarios/{name}.yaml").read_text(encoding="utf-8"))
    assert s["runner"] == "bench" and s["kind"] == kind and s["tags"] == tags
    assert (REPO / s["config"]).is_file() and (REPO / s["imperfections"]).is_file()
    assert isinstance(s["seed"], int)
    assert {k: s.get(k) for k in extra} == extra


@pytest.mark.parametrize(
    ("name", "config", "imperfections"),
    [
        ("e2e_01_ideal", IDEAL_WORLD, IDEAL),
        ("e2e_02_realistic", CONFIG, REPO / "config/imperfections/realistic.yaml"),
        ("e2e_07_gazebo", IDEAL_WORLD, IDEAL),
    ],
)
def test_ideal_profile_scenarios_run_without_friction_compensation(name, config, imperfections):
    # The ideal plant has no friction, so compensating it would inject a disturbance; the
    # realistic plant has friction and keeps the compensation of default.yaml.
    s = yaml.safe_load((REPO / f"tests/e2e/scenarios/{name}.yaml").read_text(encoding="utf-8"))
    assert (REPO / s["config"]).resolve() == config.resolve()
    assert (REPO / s["imperfections"]).resolve() == imperfections.resolve()


def test_ideal_world_config_is_default_without_compensation():
    ideal_path = IDEAL_WORLD
    default = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["cobot"]
    ideal = yaml.safe_load(ideal_path.read_text(encoding="utf-8"))["cobot"]
    urdf = (CONFIG.parent / default.pop("urdf_path")).resolve()
    assert (ideal_path.parent / ideal.pop("urdf_path")).resolve() == urdf
    log_dir = (CONFIG.parent / default.pop("log")["output_dir"]).resolve()
    assert (ideal_path.parent / ideal.pop("log")["output_dir"]).resolve() == log_dir
    # default.yaml may ship compensation off for now; ideal_world.yaml always has it off.
    assert ideal["friction_compensation"].pop("enabled") is False
    default["friction_compensation"].pop("enabled")
    assert ideal == default


def test_safety_config_is_default_with_another_goal():
    safety_path = REPO / "tests/e2e/configs/e2e_04_safety_limit.yaml"
    default = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["cobot"]
    safety = yaml.safe_load(safety_path.read_text(encoding="utf-8"))["cobot"]
    urdf = (CONFIG.parent / default.pop("urdf_path")).resolve()
    assert (safety_path.parent / safety.pop("urdf_path")).resolve() == urdf
    for cfg in (default, safety):
        cfg.pop("log")
    goal, default_goal = safety.pop("trajectory"), default.pop("trajectory")
    assert safety == default
    assert goal["goal_position_m"] == [0.1, -1.3, 0.5]
    assert goal["goal_rpy_deg"] == [0.0, -90.0, 90.0]
    assert goal["duration_s"] == default_goal["duration_s"]


def test_run_pass_and_bench_command_line(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path, thresholds=G3)])
    assert code == 0, out
    assert "T-run" in out and "PASS" in out
    (argv,) = calls
    assert argv[0] == "run" and flag(argv, "--seed") == "1"
    assert Path(flag(argv, "--config")).resolve() == CONFIG.resolve()
    assert Path(flag(argv, "--imperfections")).resolve() == IDEAL.resolve()
    assert Path(flag(argv, "--out")).resolve().is_relative_to((tmp_path / "out").resolve())


@pytest.mark.parametrize(
    ("mode", "limit_m", "expected"),
    # Fake final tip error: 10 mm (fail), 4 mm (pos4mm); crash = bench exits 3.
    [("fail", 0.002, 1), ("pos4mm", 0.005, 0), ("pos4mm", 0.002, 1), ("crash", 0.002, 1)],
)
def test_run_verdict_uses_scenario_thresholds(tmp_path, bench, mode, limit_m, expected):
    path = scenario(tmp_path, thresholds=G3 | {"final_pos_err_m": limit_m})
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], mode)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)


@pytest.mark.parametrize(("mode", "expected"), [("", 0), ("nondet", 1)])
def test_determinism_runs_twice_and_compares_bytes(tmp_path, bench, mode, expected):
    path = scenario(tmp_path, kind="determinism")
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path], mode)
    assert code == expected, out
    assert [c[0] for c in calls] == ["run", "run"]
    assert flag(calls[0], "--seed") == flag(calls[1], "--seed")
    assert flag(calls[0], "--out") != flag(calls[1], "--out")


@pytest.mark.parametrize(
    ("mode", "limit", "expected"), [("", 1000, 0), ("slow", 1000, 1), ("", 200, 1)]
)
def test_timing_parses_p95(tmp_path, bench, mode, limit, expected):
    # Fake prints p95_us=250.25 (1500 when slow): 250.25 <= 1000 passes; 1500 > 1000 and
    # 250.25 > 200 fail.
    path = scenario(tmp_path, kind="timing", steps=500, thresholds={"p95_step_us": limit})
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path], mode)
    assert code == expected, out
    (argv,) = calls
    assert argv[0] == "timing" and flag(argv, "--steps") == "500"
    assert Path(flag(argv, "--config")).resolve() == CONFIG.resolve()


@pytest.mark.parametrize(
    ("mode", "expected"), [("hold", 0), ("", 1), ("hold,tau_over", 1), ("hold,nan_tau", 1)]
)
def test_safety_checks_mode_and_torque(tmp_path, bench, mode, expected):
    # tau_over: |tau1| = 100 > tau_max 87 of joint_1 in the URDF.
    path = scenario(tmp_path, kind="safety", expect={"mode_reached": "holding"})
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], mode)
    assert code == expected, out


@pytest.mark.parametrize(("tag", "runs", "timings"), [("arm64", 5, 0), ("pr", 5, 1), (None, 5, 1)])
def test_all_filters_by_tag(tmp_path, bench, tag, runs, timings):
    # E2E-01, E2E-02, E2E-03 (two runs), E2E-04 -> 5 runs; PF-01 (tags [pr]) -> one timing.
    args = ["--all"] + (["--tag", tag] if tag else [])
    code, out, calls = run_e2e(tmp_path, bench, args)
    assert code == 0, out
    kinds = [c[0] for c in calls]
    assert kinds.count("run") == runs and kinds.count("timing") == timings
    assert any(c[0] == "run" and flag(c, "--seed") == "7" for c in calls)
    assert all(sid in out for sid in ("E2E-01", "E2E-02", "E2E-03", "E2E-04"))
    assert ("PF-01" in out) == (timings == 1)


@pytest.mark.parametrize("report", [True, False])
def test_report_unless_no_report(tmp_path, bench, report):
    # Without --no-report a report.md is written; with it, matplotlib is never imported (the
    # arm64 image has none), so a blocking stub on PYTHONPATH must not matter.
    blocker = tmp_path / "block"
    (blocker / "matplotlib").mkdir(parents=True)
    (blocker / "matplotlib" / "__init__.py").write_text("raise ImportError('blocked')\n")
    path = scenario(tmp_path, thresholds=G3)
    pythonpath = () if report else (blocker,)
    args = ["--scenario", path]
    code, out, _ = run_e2e(tmp_path, bench, args, report=report, pythonpath=pythonpath)
    assert code == 0, out
    assert bool(list((tmp_path / "out").rglob("report.md"))) == report
