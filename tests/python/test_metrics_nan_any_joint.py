"""A NaN tracking error on any joint makes rms_joint_err_rad NaN, so the run cannot pass.

Python's built-in max() skips a NaN unless it is the first element; the overall RMS must not
depend on which joint holds the NaN.
"""

import math
from pathlib import Path

import pytest
from cobot_eval import metrics

from run_csv_fixture import Run

URDF = Path(__file__).resolve().parents[2] / "sim/urdf/cobot30_5.urdf"
TH = metrics.Thresholds(1.0, 1.0, 1.0)


@pytest.mark.parametrize("joint", range(7))
def test_nan_on_any_joint_propagates_to_overall_rms(tmp_path, joint):
    run = Run(rows=20)
    run.q[5, joint] = math.nan  # tracking row
    csv = run.write(tmp_path / "nan.csv")
    m = metrics.compute_metrics(metrics.load_run(csv), metrics.read_tau_max(URDF))
    assert math.isnan(m.rms_joint_err_per_joint_rad[joint])
    assert math.isnan(m.rms_joint_err_rad)
    assert metrics.check(m, TH) != []
