"""rms_tau_ratio and the config torque limits in cobot_eval (feature joint-limits).

Design docs/design/2026-11-02-realism-2-design.md section 5:
- `rms_tau_ratio = max_j RMS(tau_j) / R_j`, the RMS taken over the tracking and finished rows
  (as rms_joint_err_rad), R the rated torque `cobot.limits.rated_torque_nm`;
- peak_tau_ratio and sat_time_s use the peak torque P `cobot.limits.peak_torque_nm`;
- the Summary CSV gains `rms_tau_ratio` right after `sat_time_s`.

API pinned here:
- `metrics.read_torque_limits(config) -> (peak, rated)`, two lists of N floats read from the
  config file's `cobot.limits`; without that section both are `read_tau_max(urdf)` of the
  config's `cobot.urdf_path` (relative to the config's directory).
- `metrics.compute_metrics(df, tau_max, t_eval_s=None, meta=None, rated_torque=...)`: the new
  keyword `rated_torque` (N values, each finite and > 0, else ValueError); the result's
  `RunMetrics.rms_tau_ratio`.
"""

import math
from pathlib import Path

import pytest
import yaml
from cobot_eval import metrics, sensitivity

from run_csv_fixture import Run

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
URDF = REPO / "src/sim/urdf/cobot30_5.urdf"
RATED = [87.0, 87.0, 52.0, 52.0, 10.0, 10.0, 10.0]
PEAK = [204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0]
SUMMARY_COLUMNS = [
    "run_id",
    "param",
    "level",
    "seed",
    "final_pos_err_m",
    "final_rot_err_rad",
    "rms_joint_err_rad",
    "peak_tau_ratio",
    "sat_time_s",
    "rms_tau_ratio",
    "held",
    "passed",
]


def known_torques() -> Run:
    """100 rows of 1 ms: 0-9 idle, 10-79 tracking, 80-89 finished, 90-99 holding.

    Joint 1: 500 N m in the idle rows and -300 N m in the holding rows (both outside the RMS
    window), 43.5 N m in the 80 tracking/finished rows. Joint 5: +3 / -3 N m alternating in the
    tracking/finished rows (RMS 3). Every other torque is 0.
    """
    run = Run(rows=100)
    run.mode = ["idle"] * 10 + ["tracking"] * 70 + ["finished"] * 10 + ["holding"] * 10
    run.tau[:10, 0] = 500.0
    run.tau[10:90, 0] = 43.5
    run.tau[90:, 0] = -300.0
    run.tau[10:90, 4] = [3.0 if k % 2 == 0 else -3.0 for k in range(80)]
    return run


def compute(tmp_path, run, rated=RATED, tau_max=PEAK):
    df = metrics.load_run(run.write(tmp_path / "run.csv"))
    return metrics.compute_metrics(df, tau_max, rated_torque=rated)


def test_rms_tau_ratio_of_constant_torque_on_joint_one(tmp_path):
    m = compute(tmp_path, known_torques())
    # Joint 1: RMS 43.5 / R 87 = 0.5; joint 5: RMS 3 / R 10 = 0.3; others 0 -> max 0.5.
    assert m.rms_tau_ratio == pytest.approx(0.5, rel=1e-12)


def test_rms_tau_ratio_takes_the_worst_joint(tmp_path):
    run = known_torques()
    # Joint 6: 0 / 8 N m alternating over the 80 rows: RMS = sqrt((0 + 64) / 2) = sqrt(32)
    # = 5.656854; / R 10 = 0.5656854 > 0.5 of joint 1.
    run.tau[10:90, 5] = [0.0 if k % 2 == 0 else 8.0 for k in range(80)]
    m = compute(tmp_path, run)
    assert m.rms_tau_ratio == pytest.approx(math.sqrt(32.0) / 10.0, rel=1e-12)


def test_rms_tau_ratio_divides_by_each_joints_rated_torque(tmp_path):
    run = known_torques()
    run.tau[10:90, 0] = 0.0
    run.tau[10:90, 4] = 0.0
    run.tau[10:90, 2] = 39.0  # joint 3: 39 / 52 = 0.75 (39 / 87 = 0.448 with joint 1's R)
    m = compute(tmp_path, run)
    assert m.rms_tau_ratio == pytest.approx(0.75, rel=1e-12)


def test_peak_and_saturation_use_the_given_peak_torque(tmp_path):
    m = compute(tmp_path, known_torques())
    # Peak over every row: 500 / P 204 = 2.4509803921568627.
    assert m.peak_tau_ratio == pytest.approx(500.0 / 204.0, rel=1e-12)
    # |tau| >= 0.999 P in the 10 idle rows (500) and the 10 holding rows (300 > 203.8); 43.5 is
    # below. 20 rows of 1 ms = 0.020 s.
    assert m.sat_time_s == pytest.approx(0.020, abs=1e-12)


def test_rms_tau_ratio_keeps_a_nan(tmp_path):
    run = known_torques()
    run.tau[40, 2] = math.nan  # tracking row, joint 3
    m = compute(tmp_path, run)
    assert math.isnan(m.rms_tau_ratio)


@pytest.mark.parametrize("rated", [RATED[:6], [*RATED[:4], 0.0, *RATED[5:]], [math.nan] * 7])
def test_bad_rated_torque_is_rejected(tmp_path, rated):
    with pytest.raises(ValueError):
        compute(tmp_path, known_torques(), rated=rated)


def test_read_torque_limits_from_default_config():
    peak, rated = metrics.read_torque_limits(CONFIG)
    assert list(peak) == PEAK
    assert list(rated) == RATED


def test_read_torque_limits_without_section_falls_back_to_urdf_effort(tmp_path):
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    data["cobot"].pop("limits", None)
    data["cobot"]["urdf_path"] = "urdf/robot.urdf"  # relative to the config's directory
    (tmp_path / "urdf").mkdir()
    (tmp_path / "urdf/robot.urdf").write_bytes(URDF.read_bytes())
    config = tmp_path / "no_limits.yaml"
    config.write_bytes(yaml.safe_dump(data).encode())
    peak, rated = metrics.read_torque_limits(config)
    effort = metrics.read_tau_max(URDF)
    assert list(peak) == effort
    assert list(rated) == effort


def test_summary_columns_put_rms_tau_ratio_after_sat_time():
    assert list(sensitivity.COLUMNS) == SUMMARY_COLUMNS


def test_conventions_document_the_summary_header():
    text = (REPO / "docs/conventions.md").read_text(encoding="utf-8")
    assert ",".join(SUMMARY_COLUMNS) in text
