"""Error metrics of one Run CSV (contract: docs/conventions.md, Run CSV)."""

import math
import re
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

ERROR_MODES = ("tracking", "finished")
SAT_FRACTION = 0.999
_JOINT_PREFIXES = ("q", "qd", "qref", "qdref", "tau")
_TASK_COLUMNS = ("px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz")


def run_columns(n: int) -> list[str]:
    """Run CSV header for `n` joints, in contract order."""
    cols = ["t_ns", "mode"]
    for prefix in _JOINT_PREFIXES:
        cols += [f"{prefix}{j}" for j in range(1, n + 1)]
    cols += list(_TASK_COLUMNS)
    cols += [f"e{k}" for k in range(1, 7)]
    cols.append("step_us")
    return cols


def load_run(path: Path) -> pd.DataFrame:
    """Read a Run CSV; raise ValueError naming the first column that breaks the contract."""
    df = pd.read_csv(path, dtype={"mode": str})
    actual = [str(c) for c in df.columns]
    n = sum(1 for c in actual if re.fullmatch(r"q\d+", c))
    if n == 0:
        raise ValueError(f"{path}: no joint position columns (q1, q2, ...)")
    expected = run_columns(n)
    for col in expected:
        if col not in actual:
            raise ValueError(f"{path}: missing column {col}")
    for i, col in enumerate(expected):
        if i >= len(actual) or actual[i] != col:
            raise ValueError(f"{path}: expected column {col} at position {i + 1}")
    if len(actual) != len(expected):
        raise ValueError(f"{path}: unexpected column {actual[len(expected)]}")
    if df.empty:
        raise ValueError(f"{path}: no data rows")
    t_ns = pd.to_numeric(df["t_ns"], errors="coerce").to_numpy(dtype=float)
    if not np.all(np.isfinite(t_ns)):
        raise ValueError(f"{path}: t_ns has a missing or non-numeric value")
    return df


@dataclass(frozen=True)
class RunMetrics:
    final_pos_err_m: float
    final_rot_err_rad: float
    rms_joint_err_rad: float
    rms_joint_err_per_joint_rad: tuple[float, ...]
    peak_tau_ratio: float
    sat_time_s: float
    held: bool
    final_t_s: float


@dataclass(frozen=True)
class Thresholds:
    final_pos_err_m: float
    final_rot_err_deg: float
    rms_joint_err_rad: float


def joint_count(df: pd.DataFrame) -> int:
    return sum(1 for c in df.columns if re.fullmatch(r"q\d+", str(c)))


def _cols(df: pd.DataFrame, names: Sequence[str]) -> np.ndarray:
    return df[list(names)].to_numpy(dtype=float)


def joint_array(df: pd.DataFrame, prefix: str) -> np.ndarray:
    """(rows, n) array of the joint columns `prefix1..prefixN`."""
    return _cols(df, [f"{prefix}{j}" for j in range(1, joint_count(df) + 1)])


def rotation_matrix(rv: np.ndarray) -> np.ndarray:
    """Rodrigues: rotation vector (rad) to a 3x3 rotation matrix."""
    theta = float(np.linalg.norm(rv))
    if theta < 1e-12:
        return np.eye(3)
    k = rv / theta
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * kx + (1.0 - math.cos(theta)) * (kx @ kx)


def rotation_angle(r: np.ndarray, rd: np.ndarray) -> float:
    """Angle (rad) of R(rd) R(r)^T."""
    m = rotation_matrix(rd) @ rotation_matrix(r).T
    skew = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
    return math.atan2(0.5 * float(np.linalg.norm(skew)), 0.5 * (float(np.trace(m)) - 1.0))


def task_errors(df: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """Per-row position error (m) and rotation error (rad) between actual and desired pose."""
    p = _cols(df, ["px", "py", "pz"])
    pd_ = _cols(df, ["pdx", "pdy", "pdz"])
    r = _cols(df, ["rx", "ry", "rz"])
    rd = _cols(df, ["rdx", "rdy", "rdz"])
    pos = np.linalg.norm(p - pd_, axis=1)
    rot = np.array([rotation_angle(r[k], rd[k]) for k in range(len(df))])
    return pos, rot


def _row_durations_s(t_ns: np.ndarray) -> np.ndarray:
    """Time each row stands for: spacing to the next row (last row repeats the previous)."""
    if len(t_ns) < 2:
        return np.zeros(len(t_ns))
    dt = np.diff(t_ns).astype(float) * 1e-9
    return np.append(dt, dt[-1])


def compute_metrics(
    df: pd.DataFrame, tau_max: Sequence[float], t_eval_s: float | None = None
) -> RunMetrics:
    n = joint_count(df)
    limits = np.asarray(tau_max, dtype=float)
    if limits.shape != (n,):
        raise ValueError(f"tau_max has {limits.size} entries, run has {n} joints")
    bad = np.flatnonzero(~(np.isfinite(limits) & (limits > 0)))
    if bad.size:
        raise ValueError(f"tau_max of joint {bad[0] + 1} is {limits[bad[0]]}, must be > 0")
    t_ns = df["t_ns"].to_numpy(dtype=np.int64)
    k = len(df) - 1
    if t_eval_s is not None:
        idx = np.flatnonzero(t_ns <= round(t_eval_s * 1e9))
        if not idx.size:
            raise ValueError(f"t_eval {t_eval_s} s is before the first sample")
        k = int(idx[-1])
    row = df.iloc[[k]]
    pos, rot = task_errors(row)

    mode = df["mode"].astype(str).to_numpy()
    err = joint_array(df, "qref") - joint_array(df, "q")
    sel = err[np.isin(mode, ERROR_MODES)]
    if sel.size:
        per_joint = tuple(float(v) for v in np.sqrt(np.mean(sel**2, axis=0)))
    else:
        per_joint = (0.0,) * n

    ratio = np.abs(joint_array(df, "tau")) / limits
    saturated = np.any(ratio >= SAT_FRACTION, axis=1)
    return RunMetrics(
        final_pos_err_m=float(pos[0]),
        final_rot_err_rad=float(rot[0]),
        rms_joint_err_rad=float(np.max(per_joint)),  # np.max keeps a NaN on any joint
        rms_joint_err_per_joint_rad=per_joint,
        peak_tau_ratio=float(ratio.max()),
        sat_time_s=float(_row_durations_s(t_ns)[saturated].sum()),
        held=bool(np.any(mode == "holding")),
        final_t_s=float(t_ns[k]) * 1e-9,
    )


def within(value: float, threshold: float) -> bool:
    """True when `value` is finite and does not exceed `threshold` (NaN/inf always fail)."""
    return math.isfinite(value) and value <= threshold


def checked_values(metrics: RunMetrics, thresholds: Thresholds) -> list[tuple[str, float, float]]:
    """(name, value, threshold) for each thresholded metric, in report order."""
    return [
        ("final_pos_err_m", metrics.final_pos_err_m, thresholds.final_pos_err_m),
        (
            "final_rot_err_deg",
            math.degrees(metrics.final_rot_err_rad),
            thresholds.final_rot_err_deg,
        ),
        ("rms_joint_err_rad", metrics.rms_joint_err_rad, thresholds.rms_joint_err_rad),
    ]


def check(metrics: RunMetrics, thresholds: Thresholds) -> list[str]:
    """One failure line per violated threshold (non-finite values fail); empty list means pass."""
    return [
        f"{name} {value:.6g} > {limit:.6g}"
        for name, value, limit in checked_values(metrics, thresholds)
        if not within(value, limit)
    ]


def read_tau_max(urdf: Path) -> list[float]:
    """Effort limits (N·m) of the revolute joints, in URDF order."""
    try:
        root = ET.parse(urdf).getroot()
    except ET.ParseError as err:
        raise ValueError(f"{urdf}: {err}") from err
    efforts = []
    for joint in root.iter("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        if limit is None or limit.get("effort") is None:
            raise ValueError(f"{urdf}: joint {joint.get('name')} has no effort limit")
        try:
            efforts.append(float(limit.get("effort")))
        except ValueError as err:
            raise ValueError(f"{urdf}: joint {joint.get('name')} effort is not a number") from err
    if not efforts:
        raise ValueError(f"{urdf}: no revolute joints")
    return efforts
