"""Monte Carlo (E2E-05) and sweep (E2E-06) branches of tools/run_e2e.py, with a fake bench.

Scenario kinds added by feature sensitivity:
- `kind: montecarlo`: `cobot_bench montecarlo --config C --imperfections I --runs <runs>
  --seed0 <seed> --out <dir>`; the runner reads `<dir>/summary.csv` and judges every run with the
  scenario thresholds (final_pos_err_m, final_rot_err_deg, rms_joint_err_rad) from the metric
  columns, not from the bench's `passed` column; pass when pass rate >= thresholds.pass_rate.
  Unless --no-report, the sensitivity report (montecarlo.md) is written under --out.
- `kind: sweep`: `cobot_bench sweep --config C --sweep <sweep> --out <dir>`, then the sensitivity
  report; pass when the bench completed and hardware_requirements.md was produced (E2E-06:
  "report produced"), whatever the requirement table says.
- Scenarios tagged `nightly` run with `--tag nightly` or `--scenario`, never with a plain
  `--all` (as `runner: gazebo` needs `--tag gazebo`), so the per-PR selection stays as it was.
Paths in a scenario (config, imperfections, sweep) are relative to the repo root.
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
HARDWARE = REPO / "config/sweeps/hardware.yaml"
G3 = {"final_pos_err_m": 0.002, "final_rot_err_deg": 1.0, "rms_joint_err_rad": 0.01}
G3_REAL = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_sweep.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, kind, **fields):
    """Scenario file with absolute paths (valid whatever base the runner resolves against)."""
    data = {"id": f"T-{kind}", "tags": ["nightly"], "runner": "bench", "kind": kind}
    data |= {"config": str(CONFIG), **fields}
    path = tmp_path / f"{data['id']}.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def sweep_file(tmp_path):
    """Two params x two levels x one seed, absolute base profile."""
    data = {
        "base_profile": str(IDEAL),
        "seeds": [1],
        "params": [
            {"name": "sensor.encoder_bits", "levels": [12, 16]},
            {"name": "actuator.command_delay_cycles", "levels": [0, 2]},
        ],
    }
    path = tmp_path / "sweep.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


def run_e2e(tmp_path, bench, args, mode="", report=True):
    """Run the runner from an unrelated cwd; returns (exit code, output, bench argv list)."""
    exe, log = bench
    env = dict(os.environ, FAKE_BENCH_MODE=mode, FAKE_BENCH_LOG=str(log), MPLBACKEND="Agg")
    env["PYTHONPATH"] = str(REPO / "eval")
    args = [str(a) for a in args] + ([] if report else ["--no-report"])
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), *args, "--bench", str(exe)]
    cmd += ["--out", str(tmp_path / "out")]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=600)
    calls = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    return proc.returncode, proc.stdout + proc.stderr, calls


def flag(argv, name):
    return argv[argv.index(name) + 1]


def found(tmp_path, name):
    return list((tmp_path / "out").rglob(name))


@pytest.mark.parametrize(
    ("name", "kind", "extra"),
    [
        (
            "e2e_05_montecarlo",
            "montecarlo",
            {
                "imperfections": "config/imperfections/realistic.yaml",
                "runs": 50,
                "thresholds": G3_REAL | {"pass_rate": 0.95},
            },
        ),
        ("e2e_06_sweep", "sweep", {"sweep": "config/sweeps/hardware.yaml", "thresholds": G3}),
    ],
)
def test_scenario_files_follow_contract(name, kind, extra):
    s = yaml.safe_load((REPO / f"tests/e2e/scenarios/{name}.yaml").read_text(encoding="utf-8"))
    assert s["runner"] == "bench" and s["kind"] == kind and s["tags"] == ["nightly"]
    assert s["config"] == "config/default.yaml"
    assert {k: s.get(k) for k in extra} == extra
    for key in ("config", "imperfections", "sweep"):
        if key in s:
            assert (REPO / s[key]).is_file(), key
    if kind == "montecarlo":
        assert isinstance(s["seed"], int)


def test_montecarlo_command_line_and_report(tmp_path, bench):
    path = scenario(
        tmp_path,
        "montecarlo",
        imperfections=str(REALISTIC),
        seed=11,
        runs=20,
        thresholds=G3_REAL | {"pass_rate": 0.95},
    )
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert "T-montecarlo" in out and "PASS" in out
    (argv,) = calls
    assert argv[0] == "montecarlo"
    assert flag(argv, "--runs") == "20" and flag(argv, "--seed0") == "11"
    assert Path(flag(argv, "--config")).resolve() == CONFIG.resolve()
    assert Path(flag(argv, "--imperfections")).resolve() == REALISTIC.resolve()
    assert Path(flag(argv, "--out")).resolve().is_relative_to((tmp_path / "out").resolve())
    assert found(tmp_path, "montecarlo.md"), out


@pytest.mark.parametrize(
    ("mode", "pass_rate", "expected"),
    [
        # Fake: 20 runs, one (two with mc_fail) 10 mm from the goal > 5 mm, all flagged passed=1.
        ("", 0.95, 0),  # 19/20 = 0.95 >= 0.95 (inclusive)
        ("mc_fail", 0.95, 1),  # 18/20 = 0.90 < 0.95: judged from the metrics, not `passed`
        ("mc_fail", 0.9, 0),  # 0.90 >= 0.90
        ("", 1.0, 1),  # 0.95 < 1.0
        ("crash", 0.5, 1),  # bench exits 3
    ],
)
def test_montecarlo_verdict_is_pass_rate_on_scenario_thresholds(
    tmp_path, bench, mode, pass_rate, expected
):
    path = scenario(
        tmp_path,
        "montecarlo",
        imperfections=str(REALISTIC),
        seed=1,
        runs=20,
        thresholds=G3_REAL | {"pass_rate": pass_rate},
    )
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], mode, report=False)
    assert code == expected, out
    assert ("FAIL" in out) == (expected == 1)


def test_sweep_command_line_and_report(tmp_path, bench):
    sweep = sweep_file(tmp_path)
    path = scenario(tmp_path, "sweep", sweep=str(sweep), thresholds=G3)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert "T-sweep" in out and "PASS" in out
    (argv,) = calls
    assert argv[0] == "sweep"
    assert Path(flag(argv, "--config")).resolve() == CONFIG.resolve()
    assert Path(flag(argv, "--sweep")).resolve() == sweep.resolve()
    assert Path(flag(argv, "--out")).resolve().is_relative_to((tmp_path / "out").resolve())
    (table,) = found(tmp_path, "hardware_requirements.md")
    text = table.read_text(encoding="utf-8")
    assert "sensor.encoder_bits" in text and "actuator.command_delay_cycles" in text


def test_sweep_fails_when_bench_fails(tmp_path, bench):
    path = scenario(tmp_path, "sweep", sweep=str(sweep_file(tmp_path)), thresholds=G3)
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], "crash")
    assert code == 1, out
    assert "FAIL" in out


def test_all_nightly_runs_e2e_05_and_e2e_06(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--all", "--tag", "nightly"])
    # Fake sweep: 1 mm per level index, so hardware.yaml levels reach 6 mm > G3 2 mm; E2E-06
    # still passes because only "report produced" decides it. Fake Monte Carlo: 1 of 50 runs
    # fails -> 0.98 >= 0.95.
    assert code == 0, out
    assert sorted(c[0] for c in calls) == ["montecarlo", "sweep"]
    mc = next(c for c in calls if c[0] == "montecarlo")
    assert flag(mc, "--runs") == "50"
    assert Path(flag(mc, "--imperfections")).resolve() == REALISTIC.resolve()
    sw = next(c for c in calls if c[0] == "sweep")
    assert Path(flag(sw, "--sweep")).resolve() == HARDWARE.resolve()
    assert "E2E-05" in out and "E2E-06" in out
    assert found(tmp_path, "hardware_requirements.md"), out


def test_plain_all_never_starts_nightly_modes(tmp_path, bench):
    # The fake fails `run`/`timing`, so the exit code here is not the point.
    _, out, calls = run_e2e(tmp_path, bench, ["--all"], report=False)
    assert not [c for c in calls if c[0] in ("montecarlo", "sweep")], out
