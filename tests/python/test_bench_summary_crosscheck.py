"""C++ Summary CSV metrics (bench/engine.hpp) against cobot_eval.metrics on the same runs.

Runs the real `cobot_bench montecarlo --keep-runs` for two short runs of the realistic profile
and recomputes every metric column from each kept Run CSV, with the torque limits of the config
(peak_tau_ratio and sat_time_s against cobot.limits.peak_torque_nm, rms_tau_ratio against
cobot.limits.rated_torque_nm; feature joint-limits). Needs the built binary: set
COBOT_BENCH to its path (tools/wsl/build.sh: $COBOT_ROOT/build/<branch>/bench), or
build/bench/cobot_bench in the repository; skipped otherwise (the CI python job has no C++).
"""

import os
import subprocess
from pathlib import Path

import pandas as pd
import pytest
from cobot_eval import metrics

REPO = Path(__file__).resolve().parents[2]
BENCH = Path(os.environ.get("COBOT_BENCH", REPO / "build/src/bench/cobot_bench"))
CONFIG = REPO / "config/default.yaml"
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
    "hold_time_s",
    "hold_reason",
    "passed",
]


@pytest.mark.skipif(not BENCH.is_file(), reason=f"cobot_bench not built ({BENCH})")
def test_summary_metrics_match_cobot_eval(tmp_path):
    out = tmp_path / "mc"
    cmd = [str(BENCH), "montecarlo", "--config", str(CONFIG)]
    cmd += ["--imperfections", str(REPO / "config/imperfections/realistic.yaml")]
    cmd += ["--runs", "2", "--seed0", "3", "--out", str(out), "--keep-runs"]
    cmd += ["--duration-s", "0.4"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, proc.stderr
    summary = pd.read_csv(out / "summary.csv", dtype={"run_id": str, "hold_reason": str})
    assert list(summary.columns) == SUMMARY_COLUMNS
    assert len(summary) == 2
    peak, rated = metrics.read_torque_limits(CONFIG)
    for _, row in summary.iterrows():
        df = metrics.load_run(out / f"run_{row['run_id']}.csv")
        m = metrics.compute_metrics(df, peak, rated_torque=rated)
        for col in (
            "final_pos_err_m",
            "final_rot_err_rad",
            "rms_joint_err_rad",
            "peak_tau_ratio",
            "rms_tau_ratio",
        ):
            assert row[col] == pytest.approx(getattr(m, col), rel=1e-9, abs=1e-12), col
        # Gravity alone needs torque: a zero ratio would mean the column is not filled.
        assert row["rms_tau_ratio"] > 0.0
        assert row["sat_time_s"] == pytest.approx(m.sat_time_s, abs=1e-9)
        assert bool(row["held"]) == m.held
        # Feature comm-timing: empty hold_time_s (NaN) and reason none when the run never held.
        assert row["hold_reason"] == m.hold_reason
        if m.held:
            assert row["hold_time_s"] == pytest.approx(m.hold_time_s, abs=1e-9)
        else:
            assert pd.isna(row["hold_time_s"])
