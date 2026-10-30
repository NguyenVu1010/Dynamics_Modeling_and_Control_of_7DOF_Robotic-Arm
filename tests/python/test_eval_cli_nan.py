"""cobot_eval CLIs on bad input (exit 2) and NaN metrics (review gaps).

report: exit 0 pass, 1 a threshold fails, 2 bad input. sensitivity: exit 0 report written,
2 bad input. Non-finite metric values never pass a threshold (metrics.within).
"""

import math
from pathlib import Path

import numpy as np
import pytest
from cobot_eval import metrics, report, sensitivity

from run_csv_fixture import Run

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "src/sim/urdf/cobot30_5.urdf"
SCENARIO = """\
id: T
thresholds:
  final_pos_err_m: 0.002
  final_rot_err_deg: 1.0
  rms_joint_err_rad: 0.01
"""
TH = metrics.Thresholds(0.002, 1.0, 0.01)


def write(path: Path, text: str) -> Path:
    path.write_bytes(text.encode())
    return path


def run_report(tmp_path: Path, csv: Path, *, urdf=URDF, scenario=SCENARIO, extra=()) -> int:
    th = write(tmp_path / "scenario.yaml", scenario)
    argv = ["--csv", str(csv), "--urdf", str(urdf), "--thresholds", str(th)]
    return report.main([*argv, "--out", str(tmp_path / "out"), *extra])


@pytest.mark.parametrize(
    "case",
    [
        "no_thresholds",
        "missing_key",
        "not_a_number",
        "nan_threshold",
        "urdf_not_xml",
        "urdf_dof_mismatch",
        "t_eval_before_start",
        "header_only",
    ],
)
def test_report_bad_input_exits_two(tmp_path, case, capsys):
    csv = Run(rows=20).write(tmp_path / "run.csv")
    kwargs = {}
    if case == "no_thresholds":
        kwargs["scenario"] = "id: T\n"
    elif case == "missing_key":
        kwargs["scenario"] = SCENARIO.replace("  rms_joint_err_rad: 0.01\n", "")
    elif case == "not_a_number":
        kwargs["scenario"] = SCENARIO.replace("0.002", "abc")
    elif case == "nan_threshold":
        kwargs["scenario"] = SCENARIO.replace("0.002", ".nan")
    elif case == "urdf_not_xml":
        kwargs["urdf"] = REPO / "tests/data/urdf/not_xml.urdf"
    elif case == "urdf_dof_mismatch":
        kwargs["urdf"] = REPO / "tests/data/urdf/planar_2link.urdf"
    elif case == "t_eval_before_start":
        kwargs["extra"] = ("--t-eval", "-1")
    else:
        csv = write(tmp_path / "run.csv", csv.read_text().splitlines()[0] + "\n")
    assert run_report(tmp_path, csv, **kwargs) == 2
    assert "error:" in capsys.readouterr().err
    assert not (tmp_path / "out" / "report.md").exists()


@pytest.mark.parametrize("case", ["missing_summary", "missing_thresholds", "bad_thresholds"])
def test_sensitivity_bad_input_exits_two(tmp_path, case, capsys):
    summary = tmp_path / "summary.csv"
    if case != "missing_summary":
        cols = ",".join(sensitivity.COLUMNS)
        write(summary, f"{cols}\n0,none,0,1,0.001,0.001,0.001,0.5,0,0,1\n")
    th = tmp_path / "scenario.yaml"
    if case == "bad_thresholds":
        write(th, "thresholds: [1, 2]\n")
    elif case == "missing_summary":
        write(th, SCENARIO)
    argv = ["--summary", str(summary), "--thresholds", str(th), "--out", str(tmp_path / "out")]
    assert sensitivity.main(argv) == 2
    assert "error:" in capsys.readouterr().err


def test_within_rejects_non_finite_values():
    assert metrics.within(0.5, 1.0)
    for bad in (math.nan, math.inf, -math.inf):
        assert not metrics.within(bad, 1.0)


def test_nan_joint_error_fails_the_check_and_the_report(tmp_path):
    run = Run(rows=50)
    run.q[10, 0] = math.nan  # tracking row, joint 1
    csv = run.write(tmp_path / "nan.csv")
    m = metrics.compute_metrics(metrics.load_run(csv), metrics.read_tau_max(URDF))
    assert math.isnan(m.rms_joint_err_per_joint_rad[0])
    assert math.isnan(m.rms_joint_err_rad)
    assert [line.split()[0] for line in metrics.check(m, TH)] == ["rms_joint_err_rad"]
    assert run_report(tmp_path, csv) == 1
    assert "FAIL" in (tmp_path / "out" / "report.md").read_text(encoding="utf-8")


def test_nan_final_pose_fails_the_check(tmp_path):
    run = Run(rows=50)
    run.p[-1, 1] = math.nan
    run.rd[-1, 2] = math.nan
    csv = run.write(tmp_path / "nan_pose.csv")
    m = metrics.compute_metrics(metrics.load_run(csv), metrics.read_tau_max(URDF))
    assert math.isnan(m.final_pos_err_m)
    assert math.isnan(m.final_rot_err_rad)
    failed = [line.split()[0] for line in metrics.check(m, TH)]
    assert failed == ["final_pos_err_m", "final_rot_err_deg"]


def test_nan_summary_row_does_not_pass(tmp_path):
    cols = ",".join(sensitivity.COLUMNS)
    rows = ["0,none,0,1,0.001,0.001,0.001,0.5,0,0,1", "1,none,0,2,nan,0.001,0.001,0.5,0,0,1"]
    rows.append("2,none,0,3,0.001,0.001,nan,0.5,0,0,1")
    path = write(tmp_path / "summary.csv", "\n".join([cols, *rows]) + "\n")
    df = sensitivity.load_summary(path)
    assert np.isnan(df["final_pos_err_m"][1])
    assert sensitivity.passes(df, TH).tolist() == [True, False, False]
    assert sensitivity.failing(df.iloc[2], TH) == ["rms_joint_err_rad"]
