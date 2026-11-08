"""E2E-15 (feature verify): `kind: verify` of tools/run_e2e.py, with a fake bench.

Contract (tests/e2e/scenarios/e2e_15_verify_quick.yaml):
- Keys: hardware, algorithm, task (paths relative to the repository root), runs, quick (bool),
  levels (list), tune_seeds, expect_verdict (pass | fail | marginal). No `config` key: the
  controller file is the hardware model's.
- One call: `cobot_bench verify --hardware H --algorithm A --task T --runs <runs> [--quick]
  --levels <levels joined by commas> --tune-seeds <tune_seeds> --out <out>/<id>`.
- Judged from <out>/<id>/report.json: every top-level key of the Verify report present, `verdict`
  equal to expect_verdict, `quick` true, and the sha256 of each file in versions.inputs equal to
  the sha256 of its bytes. Any of these failing, a bench exit code other than 0, or no
  report.json fails the scenario; the output names the scenario and, on failure, why.
- Tagged pr: the CI e2e job (`--all --tag pr`) runs it.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
E2E_15 = REPO / "tests/e2e/scenarios/e2e_15_verify_quick.yaml"
HARDWARE = "config/hardware/premium_fr3.yaml"
TASK = "config/tasks/reference_move.yaml"
REPORT_KEYS = (
    "schema",
    "disclaimer",
    "verdict",
    "quick",
    "runs",
    "seeds",
    "pass_count",
    "pass_rate",
    "wilson",
    "hold_rate",
    "worst",
    "tuning",
    "assumptions",
    "causes",
    "suggestions",
    "versions",
    "model",
    "algorithm",
    "task",
)


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_verify.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, expect="marginal", **extra):
    data = {"id": "T-verify", "tags": ["pr"], "runner": "bench", "kind": "verify"}
    data |= {"hardware": HARDWARE, "algorithm": "computed_torque", "task": TASK}
    data |= {"runs": 10, "quick": True, "levels": [40, 50], "tune_seeds": 2}
    data |= {"expect_verdict": expect, **extra}
    path = tmp_path / "T-verify.yaml"
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


def scenario_line(out):
    lines = [line for line in out.splitlines() if "T-verify" in line]
    assert lines, out
    return lines[-1]


def test_one_verify_call_with_the_scenario_flags(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    assert "PASS" in scenario_line(out)
    assert len(calls) == 1
    argv = calls[0]
    assert argv[0] == "verify"
    assert Path(flag(argv, "--hardware")).resolve() == (REPO / HARDWARE).resolve()
    assert Path(flag(argv, "--task")).resolve() == (REPO / TASK).resolve()
    assert flag(argv, "--algorithm") == "computed_torque"
    assert flag(argv, "--runs") == "10"
    assert "--quick" in argv
    assert flag(argv, "--levels") == "40,50"
    assert flag(argv, "--tune-seeds") == "2"
    out_dir = Path(flag(argv, "--out")).resolve()
    assert out_dir == (tmp_path / "out" / "T-verify").resolve()
    assert (out_dir / "report.json").is_file()


def test_paths_resolve_from_the_repository_root_not_the_cwd(tmp_path, bench):
    # The runner runs from tmp_path; config/... exists only under the repository.
    assert not (tmp_path / HARDWARE).exists()
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    assert Path(flag(calls[0], "--hardware")).is_file()
    assert Path(flag(calls[0], "--task")).is_file()


def test_another_verdict_than_expected_fails(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "verdict_pass")
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "pass" in line and "marginal" in line


def test_the_expected_verdict_may_be_any_of_the_three(tmp_path, bench):
    path = scenario(tmp_path, expect="pass")
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], "verdict_pass")
    assert code == 0, out


def test_a_report_not_in_quick_mode_fails(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "not_quick")
    assert code == 1, out
    assert "quick" in scenario_line(out)


def test_an_input_hash_that_does_not_match_fails(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "bad_hash")
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "premium_fr3.yaml" in line


def test_a_missing_report_key_fails_and_is_named(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "missing_wilson")
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "wilson" in line


@pytest.mark.parametrize("mode", ["crash", "no_report"])
def test_no_report_or_a_bench_error_fails(tmp_path, bench, mode):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], mode)
    assert code == 1, out
    assert "FAIL" in scenario_line(out)


def test_e2e_15_scenario_follows_the_brief():
    s = yaml.safe_load(E2E_15.read_text(encoding="utf-8"))
    assert s["id"] == "E2E-15"
    assert "pr" in s["tags"]
    assert s["runner"] == "bench" and s["kind"] == "verify"
    assert (REPO / s["hardware"]).resolve() == (REPO / HARDWARE).resolve()
    assert s["algorithm"] == "computed_torque"
    assert (REPO / s["task"]).resolve() == (REPO / TASK).resolve()
    assert s["quick"] is True
    assert s["runs"] == 10
    assert s["levels"] == [40, 50]
    assert s["tune_seeds"] == 2
    assert s["expect_verdict"] == "marginal"
    assert "config" not in s


def test_the_fake_report_carries_every_contract_key(tmp_path, bench):
    # Guards the fake itself: what the runner checks is what the contract lists.
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    report = json.loads((Path(flag(calls[0], "--out")) / "report.json").read_text("utf-8"))
    assert set(REPORT_KEYS) <= set(report)


@pytest.mark.parametrize("tag", ["pr", None])
def test_e2e_15_runs_under_the_pr_selection(tmp_path, bench, tag):
    # This fake serves verify only, so the other scenarios fail: the exit code is not the point.
    args = ["--all"] + (["--tag", tag] if tag else [])
    _, out, calls = run_e2e(tmp_path, bench, args)
    verify = [c for c in calls if c[0] == "verify"]
    assert len(verify) == 1, out
    assert Path(flag(verify[0], "--out")).name == "E2E-15"
    line = [row for row in out.splitlines() if row.startswith("E2E-15")]
    assert line and "PASS" in line[0], out
