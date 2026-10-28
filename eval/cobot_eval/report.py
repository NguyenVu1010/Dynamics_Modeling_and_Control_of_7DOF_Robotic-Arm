"""Markdown + PNG report of one Run CSV against scenario thresholds.

Usage: python -m cobot_eval.report --csv RUN.csv --urdf URDF --thresholds YAML --out DIR
       [--t-eval S] [--title T]
Exit code: 0 all thresholds pass, 1 a threshold fails, 2 bad input.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402
import yaml  # noqa: E402

from cobot_eval.metrics import (  # noqa: E402
    RunMetrics,
    Thresholds,
    check,
    checked_values,
    compute_metrics,
    joint_array,
    load_run,
    read_tau_max,
    task_errors,
    within,
)

EXIT_PASS, EXIT_FAIL, EXIT_BAD_INPUT = 0, 1, 2


def load_thresholds(path: Path) -> Thresholds:
    """Thresholds from a scenario file's `thresholds:` mapping; unknown keys are ignored."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("thresholds"), dict):
        raise ValueError(f"{path}: no 'thresholds' mapping")
    values = data["thresholds"]
    parsed = {}
    for key in ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad"):
        if key not in values:
            raise ValueError(f"{path}: missing threshold {key}")
        try:
            parsed[key] = float(values[key])
        except (TypeError, ValueError) as err:
            raise ValueError(f"{path}: threshold {key} is not a number") from err
        if not math.isfinite(parsed[key]):
            raise ValueError(f"{path}: threshold {key} is not finite")
    return Thresholds(**parsed)


def _verdict(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def render_markdown(title: str, csv: Path, m: RunMetrics, th: Thresholds) -> str:
    rows = checked_values(m, th)
    passed = not check(m, th)
    lines = [
        f"# {title}",
        "",
        f"Run: `{csv.name}`, evaluated at t = {m.final_t_s:.3f} s. Result: **{_verdict(passed)}**",
        "",
        "| Metric | Value | Threshold | Result |",
        "| --- | --- | --- | --- |",
    ]
    lines += [f"| {name} | {v:.6g} | {t:.6g} | {_verdict(within(v, t))} |" for name, v, t in rows]
    lines += [
        f"| final_rot_err_rad | {m.final_rot_err_rad:.6g} | | |",
        f"| peak_tau_ratio | {m.peak_tau_ratio:.4g} | | |",
        f"| sat_time_s | {m.sat_time_s:.4g} | | |",
        f"| held | {'yes' if m.held else 'no'} | | |",
        "",
        "## Joint tracking RMS (tracking and finished rows)",
        "",
        "| Joint | RMS (rad) |",
        "| --- | --- |",
    ]
    lines += [f"| {j} | {v:.6g} |" for j, v in enumerate(m.rms_joint_err_per_joint_rad, 1)]
    lines += [
        "",
        "## Plots",
        "",
        "![q](q.png)",
        "![tracking error](tracking_error.png)",
        "![tau](tau.png)",
        "![task error](task_error.png)",
        "",
    ]
    return "\n".join(lines)


def _plot_joints(t: np.ndarray, y: np.ndarray, ylabel: str, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(8, 4))
    for j in range(y.shape[1]):
        ax.plot(t, y[:, j], label=f"j{j + 1}")
    ax.set_xlabel("t (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(loc="best", fontsize="small", ncol=2)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)


def write_plots(df: pd.DataFrame, out: Path) -> None:
    t = df["t_ns"].to_numpy(dtype=float) * 1e-9
    _plot_joints(t, joint_array(df, "q"), "q (rad)", out / "q.png")
    err = joint_array(df, "qref") - joint_array(df, "q")
    _plot_joints(t, err, "q_ref - q (rad)", out / "tracking_error.png")
    _plot_joints(t, joint_array(df, "tau"), "tau (N m)", out / "tau.png")

    pos, rot = task_errors(df)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, pos * 1e3, label="position (mm)")
    ax.plot(t, np.degrees(rot), label="rotation (deg)")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("task error")
    ax.grid(True)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out / "task_error.png", dpi=100)
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cobot_eval.report", description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--t-eval", type=float, default=None)
    parser.add_argument("--title", default=None)
    args = parser.parse_args(argv)

    try:
        df = load_run(args.csv)
        tau_max = read_tau_max(args.urdf)
        thresholds = load_thresholds(args.thresholds)
        metrics = compute_metrics(df, tau_max, args.t_eval)
    except (OSError, ValueError, yaml.YAMLError, pd.errors.ParserError) as err:
        print(f"error: {err}", file=sys.stderr)
        return EXIT_BAD_INPUT

    title = args.title or f"Run report: {args.csv.stem}"
    args.out.mkdir(parents=True, exist_ok=True)
    report = render_markdown(title, args.csv, metrics, thresholds)
    (args.out / "report.md").write_bytes(report.encode("utf-8"))
    write_plots(df, args.out)

    failures = check(metrics, thresholds)
    for line in failures:
        print(f"FAIL {line}", file=sys.stderr)
    return EXIT_FAIL if failures else EXIT_PASS


if __name__ == "__main__":
    sys.exit(main())
