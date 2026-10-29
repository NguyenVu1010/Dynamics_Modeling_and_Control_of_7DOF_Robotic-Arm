"""C++ Summary CSV metrics (bench/engine.hpp) against cobot_eval.metrics on the same runs.

Runs the real `cobot_bench montecarlo --keep-runs` for two short runs of the realistic profile
and recomputes every metric column from each kept Run CSV. Needs the built binary: set
COBOT_BENCH to its path (tools/wsl/build.sh builds it under ~/cobot-build/<branch>/bench), or
build/bench/cobot_bench in the repository; skipped otherwise (the CI python job has no C++).
"""

import os
import subprocess
from pathlib import Path

import pandas as pd
import pytest
from cobot_eval import metrics

REPO = Path(__file__).resolve().parents[2]
BENCH = Path(os.environ.get("COBOT_BENCH", REPO / "build/bench/cobot_bench"))
URDF = REPO / "sim/urdf/cobot30_5.urdf"


@pytest.mark.skipif(not BENCH.is_file(), reason=f"cobot_bench not built ({BENCH})")
def test_summary_metrics_match_cobot_eval(tmp_path):
    out = tmp_path / "mc"
    cmd = [str(BENCH), "montecarlo", "--config", str(REPO / "config/default.yaml")]
    cmd += ["--imperfections", str(REPO / "config/imperfections/realistic.yaml")]
    cmd += ["--runs", "2", "--seed0", "3", "--out", str(out), "--keep-runs"]
    cmd += ["--duration-s", "0.4"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, proc.stderr
    summary = pd.read_csv(out / "summary.csv", dtype={"run_id": str})
    assert len(summary) == 2
    tau_max = metrics.read_tau_max(URDF)
    for _, row in summary.iterrows():
        m = metrics.compute_metrics(metrics.load_run(out / f"run_{row['run_id']}.csv"), tau_max)
        for col in ("final_pos_err_m", "final_rot_err_rad", "rms_joint_err_rad", "peak_tau_ratio"):
            assert row[col] == pytest.approx(getattr(m, col), rel=1e-9, abs=1e-12), col
        assert row["sat_time_s"] == pytest.approx(m.sat_time_s, abs=1e-9)
        assert bool(row["held"]) == m.held
