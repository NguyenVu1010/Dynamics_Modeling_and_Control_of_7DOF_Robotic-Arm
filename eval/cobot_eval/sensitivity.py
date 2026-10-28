"""Sensitivity and Monte Carlo report from a Summary CSV (contract: docs/conventions.md).

Usage: python -m cobot_eval.sensitivity --summary S --thresholds YAML --out DIR
Thresholds come from a scenario file's `thresholds:` mapping. Writes, for sweep rows, one PNG per
param (level -> metric, mean and max over seeds, threshold line), tornado.png and
hardware_requirements.md; for Monte Carlo rows (param = none), montecarlo.md. Exit 0 when the
report is written (failing levels are results), 2 on bad input.

A level passes when every seed meets every threshold. A param's levels are walked in severity
order: ascending, unless the smallest level fails and the largest passes (fewer encoder bits is
worse). The worst passing level is the last one before the first failure; a passing level after
it does not count.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from cobot_eval.metrics import Thresholds, within

COLUMNS = (
    "run_id",
    "param",
    "level",
    "seed",
    "final_pos_err_m",
    "final_rot_err_rad",
    "rms_joint_err_rad",
    "peak_tau_ratio",
    "sat_time_s",
    "held",
    "passed",
)
METRICS = COLUMNS[4:9]
CHECKED = ("final_pos_err_m", "final_rot_err_rad", "rms_joint_err_rad")
FAIL_ORDER = ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad")
MONTECARLO = "none"
EXIT_OK, EXIT_BAD_INPUT = 0, 2


def load_summary(path: Path) -> pd.DataFrame:
    """Read a Summary CSV; raise ValueError naming the first missing column."""
    df = pd.read_csv(path, dtype={"param": str, "run_id": str})
    for col in COLUMNS:
        if col not in df.columns:
            raise ValueError(f"{path}: missing column {col}")
    for col in ("level", *METRICS, "held", "passed"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def failing(row: pd.Series, th: Thresholds) -> list[str]:
    """Names of the thresholds a summary row violates (non-finite values fail)."""
    checks = [
        ("final_pos_err_m", row["final_pos_err_m"], th.final_pos_err_m),
        ("final_rot_err_deg", math.degrees(row["final_rot_err_rad"]), th.final_rot_err_deg),
        ("rms_joint_err_rad", row["rms_joint_err_rad"], th.rms_joint_err_rad),
    ]
    return [name for name, value, limit in checks if not within(float(value), limit)]


def passes(df: pd.DataFrame, th: Thresholds | None) -> pd.Series:
    """Per-row verdict: the thresholds when given, else the bench's `passed` column."""
    if th is None:
        return df["passed"] == 1
    return pd.Series([not failing(r, th) for _, r in df.iterrows()], index=df.index, dtype=bool)


def severity_order(df: pd.DataFrame, th: Thresholds | None) -> tuple[list[float], dict]:
    """Levels of one param from best to worst, and level -> every seed passes."""
    ok = df.assign(ok=passes(df, th)).groupby("level")["ok"].all().to_dict()
    levels = sorted(ok)
    if not ok[levels[0]] and ok[levels[-1]]:
        levels.reverse()
    return levels, ok


def sweep_params(summary: pd.DataFrame) -> list[str]:
    return [p for p in summary["param"].unique() if p != MONTECARLO]


def requirements(summary: pd.DataFrame, th: Thresholds) -> list[tuple[str, str, str, str]]:
    """(param, worst passing level, first failing level, metric that fails); '-' when none."""
    out = []
    for param in sweep_params(summary):
        df = summary[summary["param"] == param]
        levels, ok = severity_order(df, th)
        worst, first, metric = "-", "-", "-"
        for level in levels:
            if not ok[level]:
                rows = df[df["level"] == level]
                names = {n for _, r in rows.iterrows() for n in failing(r, th)}
                first = f"{level:g}"
                metric = ", ".join(n for n in FAIL_ORDER if n in names)
                break
            worst = f"{level:g}"
        out.append((param, worst, first, metric))
    return out


def tornado(
    summary: pd.DataFrame, metric: str, th: Thresholds | None = None
) -> list[tuple[str, float]]:
    """[(param, mean at the worst level - mean at level 0 or the best level)], largest first.

    Severity order comes from `th` when given, else from the `passed` column."""
    out = []
    for param in sweep_params(summary):
        df = summary[summary["param"] == param]
        levels, _ = severity_order(df, th)
        base = 0.0 if 0.0 in levels else levels[0]
        mean = df.groupby("level")[metric].mean()
        out.append((param, float(mean[levels[-1]] - mean[base])))
    return sorted(out, key=lambda pc: -abs(pc[1]))


def montecarlo_markdown(df: pd.DataFrame, th: Thresholds) -> str:
    ok = passes(df, th)
    lines = [
        "# Monte Carlo",
        "",
        f"Pass rate: {ok.mean():.4g} ({int(ok.sum())} of {len(df)} runs)",
        "",
        "| metric | p50 | p95 | max |",
        "| --- | --- | --- | --- |",
    ]
    for m in METRICS:
        v = df[m].to_numpy(dtype=float)
        p50, p95 = np.percentile(v, [50, 95])
        lines.append(f"| {m} | {p50:.6g} | {p95:.6g} | {v.max():.6g} |")
    return "\n".join(lines) + "\n"


def limit_of(metric: str, th: Thresholds) -> float:
    return {
        "final_pos_err_m": th.final_pos_err_m,
        "final_rot_err_rad": math.radians(th.final_rot_err_deg),
        "rms_joint_err_rad": th.rms_joint_err_rad,
    }[metric]


def write_plots(summary: pd.DataFrame, th: Thresholds, out: Path) -> None:
    # Imported here so that judging a summary needs no matplotlib (arm64 image).
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    for param in sweep_params(summary):
        stats = summary[summary["param"] == param].groupby("level")
        fig, axes = plt.subplots(len(CHECKED), 1, figsize=(7, 8), sharex=True)
        for ax, m in zip(axes, CHECKED, strict=True):
            ax.plot(stats[m].mean(), "o-", label="mean over seeds")
            ax.plot(stats[m].max(), "s--", label="max over seeds")
            ax.axhline(limit_of(m, th), color="red", lw=1, label="threshold")
            ax.set_ylabel(m)
            ax.legend(fontsize="small")
        axes[-1].set_xlabel(param)
        fig.tight_layout()
        fig.savefig(out / f"sweep_{param.replace('.', '_')}.png")
        plt.close(fig)
    fig, axes = plt.subplots(1, len(CHECKED), figsize=(14, 5))
    for ax, m in zip(axes, CHECKED, strict=True):
        bars = tornado(summary, m, th)[::-1]
        ax.barh([p for p, _ in bars], [c for _, c in bars])
        ax.set_title(f"{m}: worst level - baseline", fontsize="small")
    fig.tight_layout()
    fig.savefig(out / "tornado.png")
    plt.close(fig)


def requirements_markdown(rows: list[tuple[str, str, str, str]]) -> str:
    lines = [
        "# Hardware requirements",
        "",
        "Worst level of each param at which every seed still meets the thresholds.",
        "",
        "| param | worst passing level | first failing level | metric that fails |",
        "| --- | --- | --- | --- |",
    ]
    lines += [f"| `{p}` | {w} | {f} | {m} |" for p, w, f, m in rows]
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="cobot_eval.sensitivity", description=__doc__)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--thresholds", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    from cobot_eval.report import load_thresholds

    try:
        th = load_thresholds(args.thresholds)
        summary = load_summary(args.summary)
    except (OSError, ValueError) as err:
        print(f"error: {err}", file=sys.stderr)
        return EXIT_BAD_INPUT
    args.out.mkdir(parents=True, exist_ok=True)
    mc = summary[summary["param"] == MONTECARLO]
    if not mc.empty:
        (args.out / "montecarlo.md").write_text(montecarlo_markdown(mc, th), encoding="utf-8")
    if sweep_params(summary):
        write_plots(summary, th, args.out)
        text = requirements_markdown(requirements(summary, th))
        (args.out / "hardware_requirements.md").write_text(text, encoding="utf-8")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
