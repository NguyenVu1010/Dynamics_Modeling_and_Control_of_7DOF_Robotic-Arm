"""Feature comm-timing, eval side (contract: docs/conventions.md, Run CSV and Summary CSV).

- Run CSV: new last column `hold`, the reason ControlLoop entered kHolding (`none`,
  `non_finite`, `stamp_backwards`, `state_timeout`, `joint_limit`, `overspeed`), kept in every
  row until a reset.
- cobot_eval.metrics.RunMetrics gains hold_time_s (t of the first holding row minus t of the
  first row, s; None or NaN when the run never holds) and hold_reason (the `hold` value of the
  first holding row; "none" when it never holds).
- Summary CSV: `hold_time_s` (empty when none) and `hold_reason` (`none` when none) right after
  `held`; cobot_eval.sensitivity reads them.
- The sensitivity report gives the "hold rate", the fraction of runs with held = 1, per
  param/level (sweep) and per profile (Monte Carlo, montecarlo.md).
- A sweep param config.safety.hal_timeout_ms goes through tools/run_e2e.py and the report like an
  imperfection key (fake bench: a level below 8 ms holds).
"""

import math
import os
import re
import subprocess
import sys
from pathlib import Path

import pandas as pd
import pytest
import yaml
from cobot_eval import metrics, sensitivity

from run_csv_fixture import DT_NS, Run

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
PEAK = [204.0, 204.0, 107.0, 107.0, 36.0, 36.0, 36.0]
HAL = "config.safety.hal_timeout_ms"
BITS = "sensor.encoder_bits"
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
THRESHOLDS = """\
id: T-hold
thresholds:
  final_pos_err_m: 0.005
  final_rot_err_deg: 2.0
  rms_joint_err_rad: 0.02
"""


def compute(tmp_path, run):
    df = metrics.load_run(run.write(tmp_path / "run.csv"))
    return metrics.compute_metrics(df, PEAK)


def is_empty(value) -> bool:
    return value is None or (isinstance(value, float) and math.isnan(value))


# --- Run CSV and RunMetrics -------------------------------------------------------------------


def test_run_columns_end_with_hold():
    # Feature log-channels: the legacy layout keeps hold last; the current one appends eight
    # 7-wide channel blocks and cmd after it.
    legacy = metrics.legacy_run_columns(7)
    # 2 + 5 * 7 + 12 + 6 + step_us + hold = 57
    assert len(legacy) == 57
    assert legacy[-2:] == ["step_us", "hold"]
    cols = metrics.run_columns(7)
    # 57 + 8 * 7 + cmd = 114
    assert len(cols) == 114
    assert cols[:57] == legacy
    assert cols[56] == "hold"
    assert cols[-1] == "cmd"


def test_load_run_requires_the_hold_column(tmp_path):
    path = Run(rows=10).write(tmp_path / "old.csv", drop=("hold",))
    with pytest.raises(ValueError, match="hold"):
        metrics.load_run(path)


def test_hold_column_is_parsed_as_text(tmp_path):
    run = Run(rows=20)
    run.mode[10:] = ["holding"] * 10
    run.hold[10:] = ["overspeed"] * 10
    df = metrics.load_run(run.write(tmp_path / "run.csv"))
    assert list(df["hold"][:10]) == ["none"] * 10
    assert list(df["hold"][10:]) == ["overspeed"] * 10


def test_hold_time_and_reason_from_the_first_holding_row(tmp_path):
    # 100 rows from t = 5 ms in 1 ms steps; rows 40.. hold (state_timeout). First holding row at
    # t = 5 + 40 = 45 ms, first row at 5 ms: hold_time_s = 0.040.
    run = Run(rows=100)
    run.t_ns = run.t_ns + 5 * DT_NS
    run.mode[40:] = ["holding"] * 60
    run.hold[40:] = ["state_timeout"] * 60
    m = compute(tmp_path, run)
    assert m.held is True
    assert m.hold_time_s == pytest.approx(0.040, abs=1e-12)
    assert m.hold_reason == "state_timeout"


def test_reason_of_the_first_holding_row_wins(tmp_path):
    # The loop keeps its reason until a reset; should a file still change it, the entry counts.
    run = Run(rows=30)
    run.mode[12:] = ["holding"] * 18
    run.hold[12:20] = ["joint_limit"] * 8
    run.hold[20:] = ["overspeed"] * 10
    m = compute(tmp_path, run)
    assert m.hold_time_s == pytest.approx(0.012, abs=1e-12)  # 12 rows of 1 ms
    assert m.hold_reason == "joint_limit"


def test_no_hold_gives_an_empty_time_and_none(tmp_path):
    m = compute(tmp_path, Run(rows=50))
    assert m.held is False
    assert is_empty(m.hold_time_s)
    assert m.hold_reason == "none"


# --- Summary CSV -------------------------------------------------------------------------------


def test_summary_columns_put_the_hold_columns_after_held():
    assert list(sensitivity.COLUMNS) == SUMMARY_COLUMNS


def test_conventions_document_the_hold_columns():
    text = (REPO / "docs/conventions.md").read_text(encoding="utf-8")
    assert ",".join(SUMMARY_COLUMNS) in text
    for name in ("`hold`", "`hold_time_s`", "`hold_reason`"):
        assert name in text, name
    for reason in ("non_finite", "stamp_backwards", "state_timeout", "joint_limit", "overspeed"):
        assert reason in text, reason


def summary_line(run_id, param, level, seed, held, reason="state_timeout", hold_time="0.006"):
    hold = f"1,{hold_time},{reason}" if held else "0,,none"
    return f"{run_id},{param},{level},{seed},0.001,0.001,0.001,0.5,0,0.3,{hold},1"


def write_summary(path: Path, lines: list[str]) -> Path:
    path.write_bytes(("\n".join([",".join(SUMMARY_COLUMNS), *lines]) + "\n").encode())
    return path


def test_load_summary_reads_the_hold_columns(tmp_path):
    path = write_summary(
        tmp_path / "summary.csv",
        [summary_line(0, "none", 0, 1, True), summary_line(1, "none", 0, 2, False)],
    )
    df = sensitivity.load_summary(path)
    assert list(df["held"]) == [1, 0]
    assert df["hold_time_s"].iloc[0] == pytest.approx(0.006)
    assert math.isnan(df["hold_time_s"].iloc[1])
    assert list(df["hold_reason"]) == ["state_timeout", "none"]


def test_load_summary_rejects_a_missing_hold_column(tmp_path):
    cols = [c for c in SUMMARY_COLUMNS if c != "hold_reason"]
    path = tmp_path / "summary.csv"
    path.write_bytes((",".join(cols) + "\n0,none,0,1,0.001,0.001,0.001,0.5,0,0.3,0,,1\n").encode())
    with pytest.raises(ValueError, match="hold_reason"):
        sensitivity.load_summary(path)


# --- Hold rate in the report -------------------------------------------------------------------


def run_sensitivity(tmp_path, summary: Path) -> str:
    thresholds = tmp_path / "thresholds.yaml"
    thresholds.write_bytes(THRESHOLDS.encode())
    out = tmp_path / "out"
    env = dict(os.environ, MPLBACKEND="Agg")
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "src/eval"), env.get("PYTHONPATH", "")])
    cmd = [sys.executable, "-m", "cobot_eval.sensitivity", "--summary", str(summary)]
    cmd += ["--thresholds", str(thresholds), "--out", str(out)]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, proc.stderr
    return "\n".join(p.read_text(encoding="utf-8") for p in sorted(out.glob("*.md")))


def lines_with(text: str, *patterns: str) -> list[str]:
    """Lines of `text` that match every regex in `patterns`."""
    return [ln for ln in text.splitlines() if all(re.search(p, ln) for p in patterns)]


def rate(value: str) -> str:
    """A fraction such as 0.25 written as 0.25 or 25 %."""
    pct = f"{float(value) * 100:g}"
    return rf"(?<![\d.]){re.escape(value)}(?![\d])|(?<![\d.]){pct} ?%"


def level(value: str) -> str:
    return rf"(?<![\d.]){value}(\.0+)?(?![\d.])"


def test_montecarlo_report_gives_the_hold_rate(tmp_path):
    # 4 runs of one profile, 1 held: hold rate 1 / 4 = 0.25.
    lines = [summary_line(k, "none", 0, 10 + k, k == 2) for k in range(4)]
    text = run_sensitivity(tmp_path, write_summary(tmp_path / "summary.csv", lines))
    assert lines_with(text, r"(?i)hold rate", rate("0.25")), text


def test_sweep_report_gives_the_hold_rate_per_param_and_level(tmp_path):
    # 4 seeds per level. hal_timeout 5 ms: 3 of 4 held -> 0.75; 8 ms: 1 of 4 -> 0.25; 12 ms: 0.
    # encoder bits 12 / 16: never held.
    held = {"5": 3, "8": 1, "12": 0}
    lines = []
    for lv, count in held.items():
        for seed in range(4):
            lines.append(summary_line(len(lines), HAL, lv, seed + 1, seed < count))
    for lv in ("12", "16"):
        for seed in range(4):
            lines.append(summary_line(len(lines), BITS, lv, seed + 1, False))
    text = run_sensitivity(tmp_path, write_summary(tmp_path / "summary.csv", lines))
    assert re.search(r"(?i)hold rate", text), text
    assert lines_with(text, re.escape(HAL), level("5"), rate("0.75")), text
    assert lines_with(text, re.escape(HAL), level("8"), rate("0.25")), text


# --- Sweep of a controller-config param through tools/run_e2e.py (fake bench) ------------------


@pytest.fixture
def bench(tmp_path):
    exe = tmp_path / "cobot_bench"
    source = (Path(__file__).parent / "fake_bench_sweep.py").read_text(encoding="utf-8")
    exe.write_bytes(f"#!{sys.executable}\n{source}".encode())
    exe.chmod(0o755)
    return exe, tmp_path / "bench_calls.jsonl"


def test_hal_timeout_levels_change_the_hold_rate_end_to_end(tmp_path, bench):
    sweep = tmp_path / "sweep.yaml"
    sweep.write_text(
        yaml.safe_dump(
            {
                "base_profile": str(REPO / "config/imperfections/ideal.yaml"),
                "seeds": [1, 2],
                "params": [{"name": HAL, "levels": [5, 12]}],
            }
        ),
        encoding="utf-8",
    )
    data = {"id": "T-hal", "tags": ["nightly"], "runner": "bench", "kind": "sweep"}
    data |= {"config": str(CONFIG), "sweep": str(sweep)}
    data["thresholds"] = yaml.safe_load(THRESHOLDS)["thresholds"]
    scenario = tmp_path / "T-hal.yaml"
    scenario.write_text(yaml.safe_dump(data), encoding="utf-8")
    exe, log = bench
    env = dict(os.environ, FAKE_BENCH_MODE="", FAKE_BENCH_LOG=str(log), MPLBACKEND="Agg")
    env["PYTHONPATH"] = str(REPO / "src/eval")
    cmd = [sys.executable, str(REPO / "tools/run_e2e.py"), "--scenario", str(scenario)]
    cmd += ["--bench", str(exe), "--out", str(tmp_path / "out")]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, proc.stdout + proc.stderr
    summaries = list((tmp_path / "out").rglob("summary.csv"))
    assert len(summaries) == 1
    df = pd.read_csv(summaries[0], dtype={"hold_reason": str})
    assert list(df["held"]) == [1, 1, 0, 0]  # fake: below 8 ms holds
    text = "\n".join(p.read_text(encoding="utf-8") for p in (tmp_path / "out").rglob("*.md"))
    # Level 5: both seeds held (rate 1); level 12: none (rate 0).
    assert lines_with(text, re.escape(HAL), level("5"), r"(?i)(?<![\d.])1(\.0+)?(?![\d.])|100 ?%")
    assert lines_with(text, re.escape(HAL), level("12"), r"(?<![\d.])0(\.0+)?(?![\d.])|(?<!\d)0 ?%")
