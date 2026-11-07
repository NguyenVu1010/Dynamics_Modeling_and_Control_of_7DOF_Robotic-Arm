"""E2E-14 (feature drive-loop): budget_xarm7 with the drive-side position loop, `kind: profiles`.

Contract (tests/e2e/scenarios/e2e_14_budget_drive.yaml): one profile budget_xarm7 with its own
controller config config/budget_xarm7_drive.yaml, hold: fail, seeds 1-3, the E2E-12 thresholds
(proposed, awaiting owner approval), tag realism2. Run on the fake bench of
test_run_e2e_profiles.py, whose first budget run holds: a held run fails E2E-14, while E2E-12 (hold:
report) keeps reporting the same hold.
"""

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
E2E_14 = REPO / "tests/e2e/scenarios/e2e_14_budget_drive.yaml"
TH = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_profiles.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def run_e2e(tmp_path, bench, args, mode=""):
    """Run the runner from an unrelated cwd; returns (exit code, output, bench argv list)."""
    exe, log = bench
    env = dict(os.environ, FAKE_BENCH_MODE=mode, FAKE_BENCH_LOG=str(log), MPLBACKEND="Agg")
    env["PYTHONPATH"] = str(REPO / "src/eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *[str(a) for a in args]]
    cmd += ["--bench", str(exe), "--out", str(tmp_path / "out"), "--no-report"]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=600)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, calls


def flag(argv, name):
    return argv[argv.index(name) + 1]


def test_e2e_14_scenario_file():
    text = E2E_14.read_text(encoding="utf-8")
    s = yaml.safe_load(text)
    assert s["id"] == "E2E-14" and s["runner"] == "bench" and s["kind"] == "profiles"
    assert s["tags"] == ["realism2"]
    assert s["seed"] == 1 and s["runs"] == 3  # seeds 1, 2, 3
    assert list(s["profiles"]) == ["budget_xarm7"]
    p = s["profiles"]["budget_xarm7"]
    assert p["config"] == "config/budget_xarm7_drive.yaml"
    assert p["imperfections"] == "config/imperfections/budget_xarm7.yaml"
    assert (REPO / p["config"]).is_file()
    assert (REPO / p["imperfections"]).is_file()
    assert p["hold"] == "fail"
    assert p["thresholds"] == TH
    assert all(math.isfinite(v) and v > 0 for v in p["thresholds"].values())
    assert "# proposed, awaiting owner approval" in text


def test_e2e_12_keeps_the_torque_from_host_finding():
    s = yaml.safe_load((REPO / "tests/e2e/scenarios/e2e_12_profiles.yaml").read_text("utf-8"))
    budget = s["profiles"]["budget_xarm7"]
    assert budget["config"] == "config/budget_xarm7.yaml" and budget["hold"] == "report"


def test_a_held_run_fails_e2e_14(tmp_path, bench):
    # Fake bench: budget run 0 holds with state_timeout 0.87 m from the goal, runs 1-2 are clean.
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", E2E_14])
    assert code == 1, out
    assert "E2E-14" in out and "FAIL" in out and "budget_xarm7" in out, out
    assert len(calls) == 1 and calls[0][0] == "montecarlo"
    argv = calls[0]
    drive = (REPO / "config/budget_xarm7_drive.yaml").resolve()
    assert Path(flag(argv, "--config")).resolve() == drive
    assert Path(flag(argv, "--imperfections")).stem == "budget_xarm7"
    assert flag(argv, "--runs") == "3" and flag(argv, "--seed0") == "1"
    summary = tmp_path / "out" / "E2E-14" / "budget_xarm7" / "summary.csv"
    assert summary.is_file()


def test_realism2_tag_selects_e2e_14(tmp_path, bench):
    _, out, calls = run_e2e(tmp_path, bench, ["--all", "--tag", "realism2"])
    assert "E2E-14" in out, out
    configs = [Path(flag(c, "--config")).name for c in calls if c[0] == "montecarlo"]
    assert "budget_xarm7_drive.yaml" in configs, calls


@pytest.mark.parametrize("tag", [None, "pr", "nightly"])
def test_other_selections_never_start_it(tmp_path, bench, tag):
    args = ["--all"] + (["--tag", tag] if tag else [])
    _, _, calls = run_e2e(tmp_path, bench, args)
    started = [c for c in calls if c[0] == "montecarlo" and "--config" in c]
    assert not [c for c in started if Path(flag(c, "--config")).name == "budget_xarm7_drive.yaml"]
