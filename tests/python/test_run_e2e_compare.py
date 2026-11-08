"""E2E-16 and E2E-17 (feature verify-analysis): `kind: verify` with lists, with a fake bench.

Contract (tests/e2e/scenarios/e2e_16_verify_compare.yaml, e2e_17_verify_seeds.yaml):
- `hardware` is a list of model files and `algorithm` a list of names (paths relative to the
  repository root); `expect: {<model>__<algorithm>: pass | fail | marginal}` takes the place of
  `expect_verdict`; optional `expect_suggestions: {<model>__<algorithm>: [text, ...]}`, `seed0`,
  `sensitivity_seeds`, `no_sensitivity: true` (and 6a's quick, levels, tune_seeds).
- One call: `cobot_bench verify --hardware <files joined by commas> --algorithm <names joined by
  commas> --task T --runs <runs> [--seed0 n] [--sensitivity-seeds n] [--no-sensitivity]
  --out <out>/<id>`; no --quick unless the scenario says quick.
- Judged from <out>/<id>/comparison.json: the row of every `expect` pair (model and algorithm)
  has that verdict; a pair without a row fails, named. Every text of `expect_suggestions` is in
  <out>/<id>/<pair>/report.json `suggestions`. No comparison.json or a bench exit code other
  than 0 fails. The output names the scenario and, on failure, the pair and why.
- Tagged verify-nightly: `--all --tag verify-nightly` runs them; `--all`, `--all --tag pr` and
  `--all --tag nightly` do not.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "tests/e2e/scenarios"
E2E_16 = SCENARIOS / "e2e_16_verify_compare.yaml"
E2E_17 = SCENARIOS / "e2e_17_verify_seeds.yaml"
HARDWARE = [
    "config/hardware/premium_fr3.yaml",
    "config/hardware/mid_kr1205.yaml",
    "config/hardware/budget_xarm7.yaml",
]
TASK = "config/tasks/reference_move.yaml"
POSITION_LOOP = "needs a position loop in the drive (algorithm drive_position)"
EXPECT = {
    "premium_fr3__computed_torque": "pass",
    "mid_kr1205__computed_torque": "pass",
    "budget_xarm7__computed_torque": "fail",
    "budget_xarm7__drive_position": "marginal",
}


@pytest.fixture
def bench(tmp_path):
    """Executable fake bench; returns (path, file logging each call's argv)."""
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_compare.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def scenario(tmp_path, **extra):
    data = {"id": "T-compare", "tags": ["verify-nightly"], "runner": "bench", "kind": "verify"}
    data |= {"hardware": HARDWARE, "algorithm": ["computed_torque", "drive_position"]}
    data |= {"task": TASK, "runs": 100, "expect": dict(EXPECT)}
    data |= {"expect_suggestions": {"budget_xarm7__computed_torque": [POSITION_LOOP]}}
    data |= extra
    path = tmp_path / "T-compare.yaml"
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


def scenario_line(out, sid="T-compare"):
    lines = [line for line in out.splitlines() if sid in line]
    assert lines, out
    return lines[-1]


def test_one_verify_call_with_the_lists(tmp_path, bench):
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    assert "PASS" in scenario_line(out)
    assert len(calls) == 1
    argv = calls[0]
    assert argv[0] == "verify"
    files = flag(argv, "--hardware").split(",")
    assert [Path(f).resolve() for f in files] == [(REPO / h).resolve() for h in HARDWARE]
    assert flag(argv, "--algorithm") == "computed_torque,drive_position"
    assert Path(flag(argv, "--task")).resolve() == (REPO / TASK).resolve()
    assert flag(argv, "--runs") == "100"
    assert "--quick" not in argv
    assert "--seed0" not in argv
    assert "--no-sensitivity" not in argv and "--sensitivity-seeds" not in argv
    out_dir = Path(flag(argv, "--out")).resolve()
    assert out_dir == (tmp_path / "out" / "T-compare").resolve()
    assert (out_dir / "comparison.json").is_file()


def test_the_optional_flags_are_passed(tmp_path, bench):
    path = scenario(tmp_path, seed0=101, sensitivity_seeds=10)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert flag(calls[0], "--seed0") == "101"
    assert flag(calls[0], "--sensitivity-seeds") == "10"
    path = scenario(tmp_path, no_sensitivity=True)
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert code == 0, out
    assert "--no-sensitivity" in calls[-1]


def test_another_verdict_than_expected_fails_and_names_the_pair(tmp_path, bench):
    path = scenario(tmp_path)
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", path], "budget_drive_fail")
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "budget_xarm7__drive_position" in line
    assert "fail" in line and "marginal" in line


def test_a_pair_without_a_row_fails_and_is_named(tmp_path, bench):
    # pd_gravity is not in the call, so comparison.json has no row for it.
    expect = dict(EXPECT, premium_fr3__pd_gravity="pass")
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path, expect=expect)])
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "premium_fr3__pd_gravity" in line


def test_a_missing_suggestion_fails(tmp_path, bench):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], "no_suggestion")
    assert code == 1, out
    line = scenario_line(out)
    assert "FAIL" in line and "budget_xarm7__computed_torque" in line


@pytest.mark.parametrize("mode", ["crash", "no_comparison"])
def test_no_comparison_or_a_bench_error_fails(tmp_path, bench, mode):
    code, out, _ = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)], mode)
    assert code == 1, out
    assert "FAIL" in scenario_line(out)


def test_the_unlisted_pairs_are_not_judged(tmp_path, bench):
    # premium and mid drive_position are "not listed" rows; nothing expects them: PASS.
    code, out, calls = run_e2e(tmp_path, bench, ["--scenario", scenario(tmp_path)])
    assert code == 0, out
    rows = json.loads((Path(flag(calls[0], "--out")) / "comparison.json").read_text("utf-8"))
    assert "not listed" in [r["verdict"] for r in rows["rows"]]


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("path,sid", [(E2E_16, "E2E-16"), (E2E_17, "E2E-17")])
def test_the_scenarios_follow_the_brief(path, sid):
    s = load(path)
    assert s["id"] == sid
    assert s["tags"] == ["verify-nightly"]
    assert s["runner"] == "bench" and s["kind"] == "verify"
    assert [(REPO / h).resolve() for h in s["hardware"]] == [(REPO / h).resolve() for h in HARDWARE]
    assert s["algorithm"] == ["computed_torque", "drive_position"]
    assert (REPO / s["task"]).resolve() == (REPO / TASK).resolve()
    assert s["runs"] == 100
    assert not s.get("quick", False)
    assert "levels" not in s and "tune_seeds" not in s  # tuning on, the default sweep
    assert "expect_verdict" not in s and "config" not in s
    assert set(s["expect"]) == set(EXPECT)
    assert POSITION_LOOP in s["expect_suggestions"]["budget_xarm7__computed_torque"]


def test_e2e_16_expects_the_numbers_of_81():
    s = load(E2E_16)
    assert s["expect"] == EXPECT
    assert "seed0" not in s or s["seed0"] == 1


def test_e2e_17_runs_seeds_101_to_200():
    s = load(E2E_17)
    assert s["seed0"] == 101
    # The same verdicts are expected (ambiguity 18). Only budget drive_position, two runs from
    # the FAIL line, may be set to what seeds 101..200 measure (with a comment in the file).
    for pair, verdict in EXPECT.items():
        if pair != "budget_xarm7__drive_position":
            assert s["expect"][pair] == verdict, pair


def test_the_nightly_selection_runs_both_and_the_others_do_not(tmp_path, bench):
    _, out, calls = run_e2e(tmp_path, bench, ["--all", "--tag", "verify-nightly"])
    outs = sorted(Path(flag(c, "--out")).name for c in calls if c[0] == "verify")
    assert outs == ["E2E-16", "E2E-17"], out
    for sid in ("E2E-16", "E2E-17"):
        line = [row for row in out.splitlines() if row.startswith(sid)]
        assert line and "PASS" in line[0], out


@pytest.mark.parametrize("tag", ["pr", "nightly", None])
def test_pr_and_plain_selections_never_start_them(tmp_path, bench, tag):
    args = ["--all"] + (["--tag", tag] if tag else [])
    _, out, calls = run_e2e(tmp_path, bench, args)
    lists = [c for c in calls if c[0] == "verify" and "," in flag(c, "--hardware")]
    assert not lists, out


@pytest.mark.parametrize(
    "hardware,algorithm,want_hw,want_alg",
    [
        (HARDWARE[2], ["computed_torque", "drive_position"], 1, "computed_torque,drive_position"),
        (HARDWARE, "computed_torque", 3, "computed_torque"),
    ],
)
def test_either_key_may_be_a_single_value(tmp_path, bench, hardware, algorithm, want_hw, want_alg):
    # Review of #82: list mode when either key is a list; the other may stay a single value.
    path = scenario(tmp_path, hardware=hardware, algorithm=algorithm)
    _, out, calls = run_e2e(tmp_path, bench, ["--scenario", path])
    assert "Traceback" not in out, out
    assert len(calls) == 1, out
    assert len(flag(calls[0], "--hardware").split(",")) == want_hw
    assert flag(calls[0], "--algorithm") == want_alg
