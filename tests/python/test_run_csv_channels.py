"""Feature log-channels, eval side (contract: docs/conventions.md, Run CSV).

- The Run CSV gains, after `hold`, eight N-wide blocks qddref, qraw, qt, qdt, qddt, qmt, taum,
  tauf and a last column `cmd` (config, waypoints, ...): 57 + 8 * 7 + 1 = 114 columns for N = 7.
  Unknown truth (real robot) is written `nan`.
- cobot_eval.metrics.run_columns(n) is the full header, legacy_run_columns(n) the 57-column one.
  load_run accepts exactly one of the two layouts and returns the frame as read; anything else is
  a ValueError naming the first offending column; `cmd` is read as text.
"""

import os
import subprocess
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
from cobot_eval import metrics

from run_csv_fixture import Run, run_header

REPO = Path(__file__).resolve().parents[2]
BENCH = Path(os.environ.get("COBOT_BENCH", REPO / "build/src/bench/cobot_bench"))
BLOCKS = ("qddref", "qraw", "qt", "qdt", "qddt", "qmt", "taum", "tauf")


def contract_header(n: int = 7) -> list[str]:
    """The current Run CSV header, built from the conventions independently of cobot_eval."""
    cols = run_header(n)
    for prefix in BLOCKS:
        cols += [f"{prefix}{j}" for j in range(1, n + 1)]
    return cols + ["cmd"]


def write_channels(path: Path, rows: int = 10, header: list[str] | None = None) -> Path:
    """A 114-column Run CSV: the legacy fixture rows, channel values j + k / 8, qt1 nan."""
    legacy = Run(rows=rows).write(path).read_text(encoding="utf-8").splitlines()
    cols = header or contract_header()
    lines = [",".join(cols)]
    for k, line in enumerate(legacy[1:]):
        extra = [f"{j + k / 8:.17g}" for j in range(len(BLOCKS) * 7)]
        extra[2 * 7] = "nan"  # qt1: block 2 starts at 2 * 7
        lines.append(",".join([line, *extra, "config"]))
    path.write_bytes(("\n".join(lines) + "\n").encode())
    return path


@pytest.fixture
def bench():
    """The real cobot_bench (COBOT_BENCH, or build/src/bench/cobot_bench); skip when not built."""
    if not BENCH.is_file():
        pytest.skip(f"cobot_bench not built ({BENCH})")
    return BENCH


def test_column_lists_match_the_contract():
    assert metrics.run_columns(7) == contract_header(7)
    assert len(metrics.run_columns(7)) == 114
    assert metrics.legacy_run_columns(7) == run_header(7)
    assert metrics.run_columns(6)[-1] == "cmd"
    assert len(metrics.run_columns(6)) == 101  # 2 + 5 * 6 + 12 + 6 + 2 + 8 * 6 + 1


def test_load_run_accepts_the_channel_layout(tmp_path):
    df = metrics.load_run(write_channels(tmp_path / "run.csv"))
    assert list(df.columns) == contract_header()
    assert len(df) == 10
    assert df["qt1"].isna().all()
    assert df["qt2"].iloc[3] == pytest.approx(15 + 3 / 8)  # column index 15 in the extra block
    cmd = df["cmd"]
    assert pd.api.types.is_object_dtype(cmd) or pd.api.types.is_string_dtype(cmd), cmd.dtype
    assert all(isinstance(v, str) for v in df["cmd"])
    assert list(df["cmd"]) == ["config"] * 10


def test_load_run_accepts_the_legacy_layout(tmp_path):
    df = metrics.load_run(Run(rows=10).write(tmp_path / "old.csv"))
    assert list(df.columns) == run_header(7)
    assert "cmd" not in df.columns


def test_load_run_rejects_swapped_channel_names_naming_qt1(tmp_path):
    cols = contract_header()
    i, j = cols.index("qt1"), cols.index("qdt1")
    cols[i], cols[j] = cols[j], cols[i]
    with pytest.raises(ValueError, match=r"\bqt1\b"):
        metrics.load_run(write_channels(tmp_path / "swapped.csv", header=cols))


def test_load_run_rejects_a_channel_layout_without_cmd(tmp_path):
    path = write_channels(tmp_path / "no_cmd.csv")
    lines = path.read_text(encoding="utf-8").splitlines()
    path.write_bytes(("\n".join(line.rsplit(",", 1)[0] for line in lines) + "\n").encode())
    with pytest.raises(ValueError, match=r"\bcmd\b"):
        metrics.load_run(path)


def test_load_run_reads_a_cobot_bench_run(tmp_path, bench):
    out = tmp_path / "bench.csv"
    cmd = [str(bench), "run", "--config", str(REPO / "config/default.yaml")]
    cmd += ["--imperfections", str(REPO / "config/imperfections/realistic.yaml")]
    cmd += ["--seed", "1", "--out", str(out), "--duration-s", "0.2"]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, proc.stderr
    df = metrics.load_run(out)
    assert list(df.columns) == contract_header()
    assert len(df) == 200  # 0.2 s at 1000 Hz
    assert list(df["cmd"]) == ["config"] * 200
    for prefix in BLOCKS:
        values = df[[f"{prefix}{j}" for j in range(1, 8)]].to_numpy(dtype=float)
        assert np.all(np.isfinite(values)), prefix  # the bench knows the plant truth


def test_nan_cells_parse_as_nan(tmp_path):
    path = write_channels(tmp_path / "run.csv")
    df = pd.read_csv(path)  # what load_run returns must agree with a plain read
    loaded = metrics.load_run(path)
    assert np.isnan(loaded["qt1"].to_numpy(dtype=float)).all()
    assert loaded["qdt1"].to_numpy(dtype=float) == pytest.approx(df["qdt1"].to_numpy(dtype=float))
