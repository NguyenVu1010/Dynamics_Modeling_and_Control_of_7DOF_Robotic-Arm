import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from run_csv_fixture import Run

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "src/sim/urdf/cobot30_5.urdf"
PNGS = ("q.png", "tracking_error.png", "tau.png", "task_error.png")

SCENARIO = """\
id: E2E-01
runner: bench
config: config/default.yaml
imperfections: config/imperfections/ideal.yaml
seed: 1
thresholds:
  final_pos_err_m: 0.002
  final_rot_err_deg: 1.0
  rms_joint_err_rad: 0.01
  pass_rate: 0.95
  p95_step_us: 1000
"""


def make_run(offset_rad=0.0):
    run = Run(rows=200)
    s = np.linspace(0.0, 1.0, 200)[:, None]
    run.qref[:] = 0.2 * s * np.arange(1, 8)
    run.q[:] = run.qref
    run.q[:, 2] += offset_rad
    run.tau[:] = 1.0
    run.p[:] = np.hstack([0.4 + 0.1 * s, 0.1 * s, 0.5 - 0.1 * s])
    run.pd[:] = run.p
    run.mode[-20:] = ["finished"] * 20
    return run


def run_report(tmp_path, csv, *extra, thresholds=None):
    if thresholds is None:
        thresholds = tmp_path / "scenario.yaml"
        thresholds.write_bytes(SCENARIO.encode())
    out = tmp_path / "out"
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src/eval"), env.get("PYTHONPATH", "")])
    env["MPLBACKEND"] = "Agg"
    cmd = [
        sys.executable,
        "-m",
        "cobot_eval.report",
        "--csv",
        str(csv),
        "--urdf",
        str(URDF),
        "--thresholds",
        str(thresholds),
        "--out",
        str(out),
        *extra,
    ]
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True, timeout=120)
    return proc, out


def test_passing_run_exits_zero_and_writes_report(tmp_path):
    csv = make_run().write(tmp_path / "pass.csv")
    proc, out = run_report(tmp_path, csv, "--title", "Synthetic pass run")
    assert proc.returncode == 0, proc.stderr
    report = (out / "report.md").read_text(encoding="utf-8")
    assert "Synthetic pass run" in report
    assert "PASS" in report
    for name in ("final_pos_err_m", "final_rot_err", "rms_joint_err_rad"):
        assert name in report
    for png in PNGS:
        path = out / png
        assert path.is_file(), png
        assert path.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_failing_run_exits_one_and_marks_fail(tmp_path):
    csv = make_run(offset_rad=0.02).write(tmp_path / "fail.csv")
    proc, out = run_report(tmp_path, csv)
    assert proc.returncode == 1, proc.stderr
    report = (out / "report.md").read_text(encoding="utf-8")
    assert "FAIL" in report
    for png in PNGS:
        assert (out / png).is_file(), png


def test_t_eval_option_changes_evaluation_sample(tmp_path):
    run = make_run()
    run.p[150:, 0] += 0.01  # final error 10 mm after t = 0.150 s
    csv = run.write(tmp_path / "late.csv")
    proc, _ = run_report(tmp_path, csv)
    assert proc.returncode == 1, proc.stderr
    proc, out = run_report(tmp_path, csv, "--t-eval", "0.1")
    assert proc.returncode == 0, proc.stderr
    assert "PASS" in (out / "report.md").read_text(encoding="utf-8")


def test_thresholds_are_read_from_scenario(tmp_path):
    # 0.02 rad offset passes when the scenario allows 0.05 rad
    scenario = tmp_path / "loose.yaml"
    scenario.write_bytes(
        SCENARIO.replace("rms_joint_err_rad: 0.01", "rms_joint_err_rad: 0.05").encode()
    )
    csv = make_run(offset_rad=0.02).write(tmp_path / "fail.csv")
    proc, _ = run_report(tmp_path, csv, thresholds=scenario)
    assert proc.returncode == 0, proc.stderr


@pytest.mark.parametrize("bad", ["header", "missing"])
def test_bad_input_exits_two(tmp_path, bad):
    if bad == "header":
        csv = make_run().write(tmp_path / "bad.csv", drop=("qref3",))
    else:
        csv = tmp_path / "does_not_exist.csv"
    proc, _ = run_report(tmp_path, csv)
    assert proc.returncode == 2
