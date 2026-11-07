"""E2E-12 (feature profiles): `kind: profiles` of tools/run_e2e.py, with a fake bench.

Contract (tests/e2e/scenarios/e2e_12_profiles.yaml):
- `profiles` maps a profile name to {config (optional, defaults to the scenario's `config`),
  imperfections, hold: fail | report, thresholds: {final_pos_err_m, final_rot_err_deg,
  rms_joint_err_rad}}; `seed` is the first seed and `runs` the number of seeds per profile.
- For each profile, in file order: `cobot_bench montecarlo --config C --imperfections I --runs
  <runs> --seed0 <seed> --out <out>/<id>/<name>`, C the profile's config; the bench writes
  `summary.csv` there.
- Each run is judged from the Summary CSV metric columns (never from `passed`). hold: fail -> a
  held run fails the profile. hold: report -> every held run, whatever its hold_reason, counts in
  the profile's hold rate, printed as `hold rate <rate:.3g>` on the profile's line with the
  reasons, and is not judged on the thresholds; a profile with hold: report never fails on a
  hold (budget_xarm7 is a reported finding until a drive-side loop exists, owner decision
  2026-11-03). The scenario passes when every
  profile passes; the output names each profile and, on failure, the failing one.
- Tagged `realism2`: runs with --scenario or --all --tag realism2; a plain --all (and --tag pr or
  nightly) never starts it.
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
CONFIG = REPO / "config/default.yaml"
E2E_12 = REPO / "tests/e2e/scenarios/e2e_12_profiles.yaml"
NAMES = ["premium_fr3", "mid_kr1205", "budget_xarm7"]
TH = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_profiles.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, runs=3, seed=4, own_config=()):
    """Synthetic E2E-12-like scenario on placeholder profile files named like the real ones.

    Profiles named in own_config get their own controller config (tmp_path/controllers/<name>.yaml).
    """
    profiles = {}
    for name in NAMES:
        path = tmp_path / "profiles" / f"{name}.yaml"
        path.parent.mkdir(exist_ok=True)
        path.write_text("imperfections: {}\n", encoding="utf-8")
        hold = "report" if name == "budget_xarm7" else "fail"
        profiles[name] = {"imperfections": str(path), "hold": hold, "thresholds": dict(TH)}
        if name in own_config:
            cfg = tmp_path / "controllers" / f"{name}.yaml"
            cfg.parent.mkdir(exist_ok=True)
            cfg.write_text("cobot: {}\n", encoding="utf-8")
            profiles[name] = {"config": str(cfg), **profiles[name]}
    data = {"id": "T-profiles", "tags": ["realism2"], "runner": "bench", "kind": "profiles"}
    data |= {"config": str(CONFIG), "seed": seed, "runs": runs, "profiles": profiles}
    path = tmp_path / "T-profiles.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


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


def test_one_montecarlo_per_profile_in_file_order(tmp_path, bench):
    path = scenario(tmp_path, runs=3, seed=4)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert "T-profiles" in out and "PASS" in out
    assert [c[0] for c in calls] == ["montecarlo"] * 3
    for argv, name in zip(calls, NAMES, strict=True):
        assert Path(flag(argv, "--imperfections")).stem == name
        assert Path(flag(argv, "--config")).resolve() == CONFIG.resolve()
        assert flag(argv, "--runs") == "3" and flag(argv, "--seed0") == "4"
        out_dir = Path(flag(argv, "--out")).resolve()
        assert out_dir == (tmp_path / "out" / "T-profiles" / name).resolve()
        assert (out_dir / "summary.csv").is_file()
    for name in NAMES:
        assert name in out


def test_profile_config_overrides_the_scenario_config(tmp_path, bench):
    path = scenario(tmp_path, own_config=("premium_fr3", "budget_xarm7"))
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    configs = {Path(flag(c, "--imperfections")).stem: Path(flag(c, "--config")) for c in calls}
    controllers = tmp_path / "controllers"
    assert configs["premium_fr3"].resolve() == (controllers / "premium_fr3.yaml").resolve()
    assert configs["budget_xarm7"].resolve() == (controllers / "budget_xarm7.yaml").resolve()
    assert configs["mid_kr1205"].resolve() == CONFIG.resolve()


def test_budget_overspeed_hold_is_reported_not_failed(tmp_path, bench):
    # Every hold counts under hold: report: run 0 state_timeout, run 1 overspeed -> 2/3, both named.
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "budget_overspeed")
    assert code == 0, out
    budget = [line for line in out.splitlines() if "budget_xarm7" in line]
    assert any("hold rate 0.667" in line for line in budget), out
    assert any("overspeed" in line and "state_timeout" in line for line in budget), out


def test_budget_hold_rate_is_reported_not_failed(tmp_path, bench):
    # Fake: budget run 0 of 3 holds (state_timeout), 0.87 m from the goal -> hold rate 1/3,
    # printed with 3 significant digits: 0.333.
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    budget = [line for line in out.splitlines() if "budget_xarm7" in line]
    assert budget, out
    assert any("hold rate 0.333" in line for line in budget), out


def test_budget_all_held_still_passes(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "budget_all_held")
    assert code == 0, out
    assert "hold rate 1" in out


@pytest.mark.parametrize(
    ("mode", "culprit"),
    [
        ("mid_fail", "mid_kr1205"),  # 10 mm > 5 mm, judged from the metric (passed=1 in the row)
        ("premium_hold", "premium_fr3"),  # hold: fail
        ("crash", "premium_fr3"),  # bench exits 3, no summary
    ],
)
def test_failures(tmp_path, bench, mode, culprit):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], mode)
    assert code == 1, out
    assert "FAIL" in out and culprit in out, out


def test_only_tag_realism2_selects_it_in_all(tmp_path, bench):
    # --all --tag realism2 runs E2E-12 (the real file) on the fake bench.
    code, out, calls = run_e2e(tmp_path, bench, ["--all", "--tag", "realism2"])
    assert "E2E-12" in out, out
    # E2E-14 (budget_xarm7 with the drive loop) carries the tag too; its calls are pinned in
    # test_run_e2e_budget_drive.py.
    calls = [c for c in calls if Path(flag(c, "--config")).name != "budget_xarm7_drive.yaml"]
    assert [Path(flag(c, "--imperfections")).stem for c in calls] == NAMES
    assert [Path(flag(c, "--config")).name for c in calls] == [f"{n}.yaml" for n in NAMES]
    # The fake bench holds every budget run, which fails E2E-14 (no hold allowed) and so the
    # exit code; E2E-12 itself allows the hold and passes.
    row = next(line for line in out.splitlines() if line.startswith("E2E-12"))
    assert row.split()[2] == "PASS", out
    assert code == 1, out  # E2E-14 FAIL on the fake


@pytest.mark.parametrize("tag", [None, "pr", "nightly", "arm64"])
def test_other_selections_never_start_it(tmp_path, bench, tag):
    # The fake fails run/timing, so the exit code is not the point here.
    args = ["--all"] + (["--tag", tag] if tag else [])
    _, out, calls = run_e2e(tmp_path, bench, args)
    started = [c for c in calls if c[0] == "montecarlo" and "--imperfections" in c]
    assert not [c for c in started if Path(flag(c, "--imperfections")).stem in NAMES], out


def test_e2e_12_scenario_file():
    text = E2E_12.read_text(encoding="utf-8")
    s = yaml.safe_load(text)
    assert s["id"] == "E2E-12" and s["runner"] == "bench" and s["kind"] == "profiles"
    assert s["tags"] == ["realism2"]
    assert s["config"] == "config/default.yaml"
    assert s["seed"] == 1 and s["runs"] == 3  # seeds 1, 2, 3
    assert list(s["profiles"]) == NAMES
    for name, p in s["profiles"].items():
        assert p["config"] == f"config/{name}.yaml", name  # per-profile gains
        assert (REPO / p["config"]).is_file(), name
        assert p["imperfections"] == f"config/imperfections/{name}.yaml"
        assert (REPO / p["imperfections"]).is_file(), name
        assert p["hold"] == ("report" if name == "budget_xarm7" else "fail")
        assert set(p["thresholds"]) == set(TH), name
        assert all(math.isfinite(v) and v > 0 for v in p["thresholds"].values()), name
    # Design 10: the owner approves E2E thresholds; each block carries the marker.
    assert text.count("# proposed, awaiting owner approval") >= len(NAMES)


def test_e2e_12_runs_and_writes_its_summaries(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", E2E_12])
    assert code == 0, out
    assert len(calls) == 3
    for name in NAMES:
        summary = tmp_path / "out" / "E2E-12" / name / "summary.csv"
        assert summary.is_file(), name
        assert summary.read_text(encoding="utf-8").startswith("run_id,param,level,seed,")
