import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest
from cobot_eval.metrics import (
    RunMetrics,
    Thresholds,
    check,
    compute_metrics,
    load_run,
    read_tau_max,
)

from run_csv_fixture import Run, run_header

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "sim/urdf/cobot30_5.urdf"
TAU_MAX = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
LOOSE = Thresholds(final_pos_err_m=0.002, final_rot_err_deg=1.0, rms_joint_err_rad=0.01)


def smooth_run(rows=100, n=7):
    """q = qref, p = pd, r = rd on a non-trivial trajectory, small torques."""
    run = Run(rows=rows, n=n)
    s = np.linspace(0.0, 1.0, rows)[:, None]
    run.q[:] = 0.3 * s * np.arange(1, n + 1)
    run.qref[:] = run.q
    run.qd[:] = 0.3 * np.arange(1, n + 1)
    run.qdref[:] = run.qd
    run.tau[:] = 0.1
    run.p[:] = np.hstack([0.4 + 0.1 * s, 0.1 * s, 0.5 - 0.1 * s])
    run.pd[:] = run.p
    run.r[:] = np.hstack([0.2 * s, -0.1 * s, 0.3 * s])
    run.rd[:] = run.r
    run.mode[-10:] = ["finished"] * 10
    return run


def metrics_of(tmp_path, run, tau_max=TAU_MAX, t_eval_s=None, name="run.csv"):
    df = load_run(run.write(tmp_path / name))
    return compute_metrics(df, tau_max, t_eval_s)


# --- load_run -------------------------------------------------------------------------------


def test_load_run_returns_all_contract_columns(tmp_path):
    df = load_run(smooth_run().write(tmp_path / "run.csv"))
    assert list(df.columns) == run_header(7)
    assert len(df) == 100
    assert df["mode"].iloc[-1] == "finished"
    assert df["t_ns"].iloc[1] - df["t_ns"].iloc[0] == 1_000_000


def test_load_run_missing_column_raises_naming_it(tmp_path):
    path = smooth_run().write(tmp_path / "bad.csv", drop=("qref3",))
    with pytest.raises(ValueError, match="qref3"):
        load_run(path)


def test_load_run_missing_task_column_raises_naming_it(tmp_path):
    path = smooth_run().write(tmp_path / "bad.csv", drop=("rdz",))
    with pytest.raises(ValueError, match="rdz"):
        load_run(path)


def test_load_run_infers_joint_count(tmp_path):
    run = smooth_run(n=6)
    df = load_run(run.write(tmp_path / "six.csv"))
    assert list(df.columns) == run_header(6)
    m = compute_metrics(df, TAU_MAX[:6])
    assert len(m.rms_joint_err_per_joint_rad) == 6


# --- compute_metrics ------------------------------------------------------------------------


def test_metrics_types_are_frozen_dataclasses():
    assert dataclasses.is_dataclass(RunMetrics)
    assert dataclasses.is_dataclass(Thresholds)
    with pytest.raises(dataclasses.FrozenInstanceError):
        LOOSE.final_pos_err_m = 1.0


def test_perfect_tracking_has_zero_errors(tmp_path):
    m = metrics_of(tmp_path, smooth_run())
    assert m.final_pos_err_m == pytest.approx(0.0, abs=1e-12)
    assert m.final_rot_err_rad == pytest.approx(0.0, abs=1e-7)
    assert m.rms_joint_err_rad == pytest.approx(0.0, abs=1e-12)
    assert len(m.rms_joint_err_per_joint_rad) == 7
    assert all(v == pytest.approx(0.0, abs=1e-12) for v in m.rms_joint_err_per_joint_rad)
    assert isinstance(m.rms_joint_err_per_joint_rad, tuple)
    assert m.peak_tau_ratio == pytest.approx(0.1 / 12.0)
    assert m.sat_time_s == pytest.approx(0.0)
    assert m.held is False
    assert m.final_t_s == pytest.approx(0.099)
    assert check(m, LOOSE) == []


def test_constant_offset_on_joint_3_only(tmp_path):
    run = smooth_run()
    run.q[:, 2] = run.qref[:, 2] - 0.02
    m = metrics_of(tmp_path, run)
    assert m.rms_joint_err_rad == pytest.approx(0.02)
    per = m.rms_joint_err_per_joint_rad
    assert per[2] == pytest.approx(0.02)
    for j in (0, 1, 3, 4, 5, 6):
        assert per[j] == pytest.approx(0.0, abs=1e-12)


def test_rms_uses_only_tracking_and_finished_rows(tmp_path):
    run = smooth_run()
    run.mode[:5] = ["idle"] * 5
    run.q[:5, 0] = run.qref[:5, 0] + 1.0  # large error while idle: ignored
    run.q[5:, 0] = run.qref[5:, 0] + 0.01
    m = metrics_of(tmp_path, run)
    assert m.rms_joint_err_per_joint_rad[0] == pytest.approx(0.01)
    assert m.rms_joint_err_rad == pytest.approx(0.01)


def test_rms_is_root_mean_square(tmp_path):
    run = smooth_run(rows=4)
    run.mode[:] = ["tracking"] * 4
    run.q[:, 1] = run.qref[:, 1] - np.array([0.0, 0.03, 0.0, -0.04])
    m = metrics_of(tmp_path, run)
    assert m.rms_joint_err_per_joint_rad[1] == pytest.approx(math.sqrt((0.03**2 + 0.04**2) / 4))


def test_final_position_error_is_euclidean(tmp_path):
    run = smooth_run()
    run.p[-1] = run.pd[-1] + np.array([0.003, 0.0, 0.004])
    m = metrics_of(tmp_path, run)
    assert m.final_pos_err_m == pytest.approx(0.005)


def test_final_rotation_error_about_z(tmp_path):
    run = smooth_run()
    run.r[-1] = [0.0, 0.0, 0.0]
    run.rd[-1] = [0.0, 0.0, 0.01]
    m = metrics_of(tmp_path, run)
    assert m.final_rot_err_rad == pytest.approx(0.01, rel=1e-6)


def test_final_rotation_error_is_relative_angle(tmp_path):
    # R(rd) R(r)^T with both rotations about the same axis -> angle difference
    run = smooth_run()
    axis = np.array([1.0, 2.0, 2.0]) / 3.0
    run.r[-1] = 1.2 * axis
    run.rd[-1] = 1.25 * axis
    m = metrics_of(tmp_path, run)
    assert m.final_rot_err_rad == pytest.approx(0.05, rel=1e-6)


def test_final_rotation_error_noncommuting(tmp_path):
    # r = 90 deg about x, rd = 90 deg about y: R_d R^T is a 120 deg rotation
    run = smooth_run()
    run.r[-1] = [math.pi / 2, 0.0, 0.0]
    run.rd[-1] = [0.0, math.pi / 2, 0.0]
    m = metrics_of(tmp_path, run)
    assert m.final_rot_err_rad == pytest.approx(2.0 * math.pi / 3.0, rel=1e-6)


def test_t_eval_picks_row_at_or_before(tmp_path):
    run = smooth_run()
    k = np.arange(100)
    run.p[:, 0] = run.pd[:, 0] + 1e-4 * k  # error equals row index * 0.1 mm
    path = run.write(tmp_path / "run.csv")
    df = load_run(path)
    m = compute_metrics(df, TAU_MAX, 0.0505)
    assert m.final_pos_err_m == pytest.approx(50e-4)
    assert m.final_t_s == pytest.approx(0.050)
    m = compute_metrics(df, TAU_MAX, 0.050)
    assert m.final_pos_err_m == pytest.approx(50e-4)
    m = compute_metrics(df, TAU_MAX, 0.0509999)
    assert m.final_pos_err_m == pytest.approx(50e-4)
    m = compute_metrics(df, TAU_MAX)
    assert m.final_pos_err_m == pytest.approx(99e-4)
    assert m.final_t_s == pytest.approx(0.099)
    m = compute_metrics(df, TAU_MAX, 10.0)
    assert m.final_pos_err_m == pytest.approx(99e-4)


def test_saturation_for_five_rows(tmp_path):
    run = smooth_run()
    run.tau[40:45, 4] = TAU_MAX[4]
    m = metrics_of(tmp_path, run)
    assert m.peak_tau_ratio == pytest.approx(1.0)
    assert m.sat_time_s == pytest.approx(0.005)


def test_saturation_counts_negative_torque_and_any_joint(tmp_path):
    run = smooth_run()
    run.tau[10:13, 0] = -TAU_MAX[0]
    run.tau[12:15, 6] = 0.9995 * TAU_MAX[6]  # >= 0.999 tau_max counts; row 12 overlaps
    m = metrics_of(tmp_path, run)
    assert m.peak_tau_ratio == pytest.approx(1.0)
    assert m.sat_time_s == pytest.approx(0.005)


def test_peak_tau_ratio_uses_per_joint_limit(tmp_path):
    run = smooth_run()
    run.tau[:, 0] = 43.5  # 0.5 of 87
    run.tau[30, 5] = -9.0  # 0.75 of 12
    m = metrics_of(tmp_path, run)
    assert m.peak_tau_ratio == pytest.approx(0.75)
    assert m.sat_time_s == pytest.approx(0.0)


def test_holding_row_sets_held(tmp_path):
    run = smooth_run()
    run.mode[60] = "holding"
    m = metrics_of(tmp_path, run)
    assert m.held is True


# --- check ----------------------------------------------------------------------------------


def test_check_reports_one_line_per_violation(tmp_path):
    run = smooth_run()
    run.q[:, 2] = run.qref[:, 2] + 0.02
    run.p[-1] = run.pd[-1] + np.array([0.003, 0.0, 0.004])
    m = metrics_of(tmp_path, run)
    failures = check(m, LOOSE)
    assert len(failures) == 2
    assert any("rms_joint_err_rad" in line for line in failures)
    assert any("final_pos_err_m" in line for line in failures)


def test_check_rotation_threshold_in_degrees(tmp_path):
    run = smooth_run()
    run.r[-1] = [0.0, 0.0, 0.0]
    run.rd[-1] = [0.0, 0.0, 0.01]  # 0.573 deg
    m = metrics_of(tmp_path, run)
    assert check(m, LOOSE) == []
    tight = Thresholds(final_pos_err_m=0.002, final_rot_err_deg=0.5, rms_joint_err_rad=0.01)
    failures = check(m, tight)
    assert len(failures) == 1
    assert "final_rot_err" in failures[0]


# --- read_tau_max ---------------------------------------------------------------------------


def test_read_tau_max_from_committed_urdf():
    assert read_tau_max(URDF) == pytest.approx(TAU_MAX)
    assert len(read_tau_max(URDF)) == 7
