"""Optional metric `stop_pos_err_m` of cobot_eval (feature bench-waypoints, design 2026-10-29
section 7, E2E-10).

Contract pinned here:
- `metrics.stop_pos_err_m(df, meta) -> float | None`: `meta` is the parsed
  `<run.csv>.meta.yaml` mapping. With the keys `waypoint_node_times_s`, `waypoint_stop` and
  `waypoint_positions_m` (written by `cobot_bench run --waypoints`), the maximum over the stop
  waypoints i of |p_tip - waypoint_positions_m[i]|, p_tip = (px, py, pz) of the Run CSV row
  whose t_ns * 1e-9 is nearest to waypoint_node_times_s[i] + 0.2 s (settle; the last row when
  that time is past the end). None when a key is missing (or meta is None/empty) or no
  waypoint stops.
- `compute_metrics(df, tau_max, t_eval_s=None, meta=None)` sets `RunMetrics.stop_pos_err_m`
  (default None: not computed).
- `Thresholds(..., stop_pos_err_m=None)`: optional; when set, `check` judges the metric and
  reports "stop_pos_err_m <value> > <limit>"; a set threshold with a metric that was not
  computed is a failure naming stop_pos_err_m (a waypoint scenario must not pass silently).
- `cobot_eval.report` reads `<csv>.meta.yaml` next to the CSV and shows stop_pos_err_m in
  report.md when it is computed; the scenario threshold key `stop_pos_err_m` is optional.
"""

import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml
from cobot_eval import metrics

from run_csv_fixture import Run

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "src/sim/urdf/cobot30_5.urdf"
TAU_MAX = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]
G3_REAL = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}

# Synthetic run: 3001 rows at 1 ms (t = 0 .. 3.000 s), tip at (-1, -1, -1) everywhere except
# the two rows read after the settle time of the stop waypoints.
# Waypoints: node times 0.5, 1.0, 1.8, 2.5 s; stop flags F, T, F, T.
#   waypoint 1 (stop): node 1.0 s -> row t = 1.0 + 0.2 = 1.200 s (k = 1200), target
#     (0.3, 0, 0), tip (0.303, 0.004, 0): error sqrt(0.003^2 + 0.004^2) = 0.005 m.
#   waypoint 3 (stop): node 2.5 s -> row t = 2.700 s (k = 2700), target (0, 0.2, 0),
#     tip (0, 0.2, 0.002): error 0.002 m.
#   -> stop_pos_err_m = max(0.005, 0.002) = 0.005 m.
# Waypoints 0 and 2 do not stop; their targets (5, 5, 5) are far from every tip, so including
# them would give >= |(5,5,5) - (-1,-1,-1)| = 6 sqrt(3) ~ 10.39 m. Reading the row at the node
# time itself (no settle) gives |(-1,-1,-1) - (0.3,0,0)| = sqrt(1.3^2 + 1 + 1) ~ 1.814 m.
META = {
    "config": "config/default.yaml",
    "imperfections": "config/imperfections/realistic.yaml",
    "seed": 7,
    "waypoints_file": "tests/e2e/waypoints/e2e_10.yaml",
    "waypoint_node_times_s": [0.5, 1.0, 1.8, 2.5],
    "waypoint_stop": [False, True, False, True],
    "waypoint_positions_m": [[5.0, 5.0, 5.0], [0.3, 0.0, 0.0], [5.0, 5.0, 5.0], [0.0, 0.2, 0.0]],
}
WAYPOINT_KEYS = ("waypoint_node_times_s", "waypoint_stop", "waypoint_positions_m")


def make_run():
    run = Run(rows=3001)
    run.p[:] = -1.0
    run.p[1200] = [0.303, 0.004, 0.0]
    run.p[2700] = [0.0, 0.2, 0.002]
    run.pd[:] = run.p
    return run


@pytest.fixture
def df(tmp_path):
    return metrics.load_run(make_run().write(tmp_path / "run.csv"))


def test_stop_pos_err_by_hand(df):
    assert metrics.stop_pos_err_m(df, META) == pytest.approx(0.005, abs=1e-12)


def test_stop_pos_err_uses_the_nearest_row(df):
    # Node 1.0004 s -> 1.2004 s, nearest row k = 1200 (0.4 ms away; k = 1201 is 0.6 ms away).
    # Node 2.4996 s -> 2.6996 s, nearest row k = 2700. Same rows as by hand: 0.005 m.
    meta = META | {"waypoint_node_times_s": [0.5, 1.0004, 1.8, 2.4996]}
    assert metrics.stop_pos_err_m(df, meta) == pytest.approx(0.005, abs=1e-12)


def test_stop_pos_err_after_the_end_uses_the_last_row(tmp_path):
    # Node 2.9 s -> 3.1 s is past the last row (3.000 s): the last row is the nearest.
    # Tip there (0, 0, 0.003), target (0, 0, 0) -> 0.003 m.
    run = make_run()
    run.p[-1] = [0.0, 0.0, 0.003]
    df = metrics.load_run(run.write(tmp_path / "run.csv"))
    meta = META | {
        "waypoint_node_times_s": [2.9],
        "waypoint_stop": [True],
        "waypoint_positions_m": [[0.0, 0.0, 0.0]],
    }
    assert metrics.stop_pos_err_m(df, meta) == pytest.approx(0.003, abs=1e-12)


@pytest.mark.parametrize("missing", WAYPOINT_KEYS)
def test_stop_pos_err_absent_without_meta_keys(df, missing):
    meta = {k: v for k, v in META.items() if k != missing}
    assert metrics.stop_pos_err_m(df, meta) is None


@pytest.mark.parametrize("meta", [None, {}, {"config": "c", "imperfections": "i", "seed": 1}])
def test_stop_pos_err_absent_for_runs_without_waypoints(df, meta):
    assert metrics.stop_pos_err_m(df, meta) is None


def test_stop_pos_err_absent_without_stop_waypoints(df):
    assert metrics.stop_pos_err_m(df, META | {"waypoint_stop": [False] * 4}) is None


def test_compute_metrics_carries_stop_pos_err(df):
    assert metrics.compute_metrics(df, TAU_MAX).stop_pos_err_m is None
    m = metrics.compute_metrics(df, TAU_MAX, meta=META)
    assert m.stop_pos_err_m == pytest.approx(0.005, abs=1e-12)


def thresholds(stop=None):
    return metrics.Thresholds(
        final_pos_err_m=1.0, final_rot_err_deg=180.0, rms_joint_err_rad=1.0, stop_pos_err_m=stop
    )


def test_thresholds_without_stop_key_ignore_the_metric(df):
    # Default threshold None: an existing scenario without the key is judged as before.
    assert metrics.Thresholds(1.0, 180.0, 1.0).stop_pos_err_m is None
    m = metrics.compute_metrics(df, TAU_MAX, meta=META)
    assert metrics.check(m, thresholds()) == []


@pytest.mark.parametrize(("limit", "fails"), [(0.006, False), (0.004, True)])
def test_check_judges_stop_pos_err(df, limit, fails):
    # 0.005 <= 0.006 passes; 0.005 > 0.004 fails.
    m = metrics.compute_metrics(df, TAU_MAX, meta=META)
    failures = metrics.check(m, thresholds(limit))
    assert bool(failures) == fails
    if fails:
        (line,) = failures
        assert line.startswith("stop_pos_err_m")


def test_check_fails_when_threshold_set_but_metric_not_computed(df):
    m = metrics.compute_metrics(df, TAU_MAX)
    failures = metrics.check(m, thresholds(0.005))
    assert any("stop_pos_err_m" in f for f in failures), failures


def test_nan_tip_fails_the_stop_check(tmp_path):
    run = make_run()
    run.p[1200] = [math.nan, 0.0, 0.0]
    df = metrics.load_run(run.write(tmp_path / "run.csv"))
    m = metrics.compute_metrics(df, TAU_MAX, meta=META)
    assert any(f.startswith("stop_pos_err_m") for f in metrics.check(m, thresholds(0.005)))


def run_report(tmp_path, with_meta, stop_threshold):
    csv = make_run().write(tmp_path / "run.csv")
    if with_meta:
        (tmp_path / "run.csv.meta.yaml").write_bytes(yaml.safe_dump(META).encode())
    th = dict(G3_REAL)
    if stop_threshold is not None:
        th["stop_pos_err_m"] = stop_threshold
    scenario = tmp_path / "scenario.yaml"
    scenario.write_bytes(yaml.safe_dump({"id": "E2E-10", "thresholds": th}).encode())
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src/eval"), env.get("PYTHONPATH", "")])
    env["MPLBACKEND"] = "Agg"
    out = tmp_path / "out"
    cmd = [sys.executable, "-m", "cobot_eval.report", "--csv", str(csv), "--urdf", str(URDF)]
    cmd += ["--thresholds", str(scenario), "--out", str(out)]
    proc = subprocess.run(cmd, cwd=REPO, env=env, capture_output=True, text=True, timeout=120)
    report = out / "report.md"
    return proc, report.read_text(encoding="utf-8") if report.is_file() else ""


@pytest.mark.parametrize(("limit", "code"), [(0.006, 0), (0.004, 1)])
def test_report_shows_stop_pos_err(tmp_path, limit, code):
    # Final errors are 0 (pd = p), joint errors 0: only the stop threshold can fail.
    proc, report = run_report(tmp_path, with_meta=True, stop_threshold=limit)
    assert proc.returncode == code, proc.stderr
    (line,) = [ln for ln in report.splitlines() if ln.startswith("| stop_pos_err_m")]
    assert "0.005" in line
    assert ("FAIL" in line) == (code == 1)


def test_report_without_meta_has_no_stop_row(tmp_path):
    proc, report = run_report(tmp_path, with_meta=False, stop_threshold=None)
    assert proc.returncode == 0, proc.stderr
    assert "stop_pos_err_m" not in report


def test_report_shows_stop_pos_err_without_threshold(tmp_path):
    proc, report = run_report(tmp_path, with_meta=True, stop_threshold=None)
    assert proc.returncode == 0, proc.stderr
    assert "stop_pos_err_m" in report


def test_meta_numbers_by_hand_are_consistent():
    # Guard on the fixture arithmetic above.
    assert math.hypot(0.003, 0.004) == pytest.approx(0.005, abs=1e-15)
    assert np.linalg.norm(np.array([5.0, 5.0, 5.0]) + 1.0) == pytest.approx(6 * math.sqrt(3))
