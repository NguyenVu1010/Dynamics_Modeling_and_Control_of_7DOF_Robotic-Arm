"""Run CSV `standby` rows (feature loop-external): documented, and not tracking error."""

from pathlib import Path

import pytest
from cobot_eval.metrics import ERROR_MODES, compute_metrics, load_run

from run_csv_fixture import Run

REPO = Path(__file__).resolve().parents[2]
TAU_MAX = [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0]


def test_error_modes_stay_tracking_and_finished():
    assert ERROR_MODES == ("tracking", "finished")


def test_standby_rows_are_not_tracking_error(tmp_path):
    run = Run(rows=20)
    run.mode[:5] = ["standby"] * 5
    run.mode[15:] = ["standby"] * 5
    run.q[:5, 0] = 1.0  # large error while in standby: ignored
    run.q[15:, 0] = -1.0
    run.q[5:15, 0] = 0.01  # q_ref = 0: 0.01 rad while tracking
    m = compute_metrics(load_run(run.write(tmp_path / "run.csv")), TAU_MAX)
    assert m.rms_joint_err_per_joint_rad[0] == pytest.approx(0.01)
    assert not m.held


def test_conventions_document_the_standby_mode_and_the_waypoint_file():
    text = (REPO / "docs/conventions.md").read_text(encoding="utf-8")
    mode_rows = [line for line in text.splitlines() if line.startswith("| `mode` |")]
    assert len(mode_rows) == 1
    assert "`standby`" in mode_rows[0]
    assert "Waypoint file" in text
