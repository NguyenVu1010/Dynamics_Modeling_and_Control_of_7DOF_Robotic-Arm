"""Synthetic Run CSV writer for the eval tests (contract: docs/conventions.md, Run CSV)."""

from pathlib import Path

import numpy as np

DT_NS = 1_000_000  # 1 ms


def run_header(n: int = 7) -> list[str]:
    cols = ["t_ns", "mode"]
    for prefix in ("q", "qd", "qref", "qdref", "tau"):
        cols += [f"{prefix}{j}" for j in range(1, n + 1)]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    cols += [f"e{k}" for k in range(1, 7)]
    cols += ["step_us"]
    return cols


class Run:
    """Arrays for a run of `rows` samples, t_ns = k * 1 ms, every row in mode `tracking`.

    All signals are zero by default; tests overwrite the arrays they care about.
    """

    def __init__(self, rows: int = 100, n: int = 7) -> None:
        self.n = n
        self.t_ns = np.arange(rows, dtype=np.int64) * DT_NS
        self.mode = ["tracking"] * rows
        self.q = np.zeros((rows, n))
        self.qd = np.zeros((rows, n))
        self.qref = np.zeros((rows, n))
        self.qdref = np.zeros((rows, n))
        self.tau = np.zeros((rows, n))
        self.p = np.zeros((rows, 3))
        self.r = np.zeros((rows, 3))
        self.pd = np.zeros((rows, 3))
        self.rd = np.zeros((rows, 3))
        self.e = np.zeros((rows, 6))
        self.step_us = np.zeros(rows)

    def write(self, path: Path, drop: tuple[str, ...] = ()) -> Path:
        """Write the CSV; columns named in `drop` are left out (to build bad headers)."""
        cols = run_header(self.n)
        keep = [i for i, c in enumerate(cols) if c not in drop]
        lines = [",".join(cols[i] for i in keep)]
        for k in range(len(self.t_ns)):
            vals = [str(int(self.t_ns[k])), self.mode[k]]
            for arr in (self.q, self.qd, self.qref, self.qdref, self.tau):
                vals += [f"{v:.17g}" for v in arr[k]]
            for arr in (self.p, self.r, self.pd, self.rd, self.e):
                vals += [f"{v:.17g}" for v in arr[k]]
            vals.append(f"{self.step_us[k]:.17g}")
            lines.append(",".join(vals[i] for i in keep))
        path.write_bytes(("\n".join(lines) + "\n").encode())
        return path
