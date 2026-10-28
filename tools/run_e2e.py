"""End-to-end scenario runner (design 11.3): runs cobot_bench and judges the result.

Usage: python tools/run_e2e.py (--scenario YAML | --all [--tag T]) --bench PATH --out DIR
       [--no-report]
Scenario paths are relative to the repository root. kind: run (thresholds on the Run CSV),
determinism (two runs, identical bytes), safety (expect.mode_reached, |tau| <= tau_max and
finite), timing (p95 of `cobot_bench timing` <= thresholds.p95_step_us), montecarlo (share of
runs in the summary meeting the thresholds >= thresholds.pass_rate), sweep (bench completes and
the sensitivity report with hardware_requirements.md is produced). runner: gazebo scenarios are
skipped unless --tag gazebo; scenarios tagged nightly run only with --tag nightly or --scenario.
Exit 0 all pass, 1 any fails, 2 bad arguments.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "eval"))

from cobot_eval import sensitivity  # noqa: E402
from cobot_eval.metrics import (  # noqa: E402
    Thresholds,
    check,
    compute_metrics,
    joint_array,
    load_run,
    read_tau_max,
)

SCENARIO_DIR = REPO / "tests/e2e/scenarios"
THRESHOLD_KEYS = ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad")


def repo_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPO / path


def urdf_of(s: dict) -> Path:
    """URDF named by the scenario's config (path relative to the config file)."""
    config = repo_path(s["config"])
    return config.parent / yaml.safe_load(config.read_text(encoding="utf-8"))["cobot"]["urdf_path"]


def tau_max_of(s: dict) -> np.ndarray:
    return np.asarray(read_tau_max(urdf_of(s)), dtype=float)


def bench(args: argparse.Namespace, *cmd: str) -> subprocess.CompletedProcess:
    proc = subprocess.run([str(args.bench), *cmd], capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"bench exit {proc.returncode}: {proc.stderr.strip()}")
    return proc


def thresholds_of(s: dict) -> Thresholds:
    return Thresholds(*(float(s["thresholds"][k]) for k in THRESHOLD_KEYS))


def sensitivity_report(s: dict, summary: Path, out: Path) -> None:
    (out / "scenario.yaml").write_text(yaml.safe_dump(s), encoding="utf-8")
    args = ["--summary", str(summary), "--thresholds", str(out / "scenario.yaml")]
    if sensitivity.main([*args, "--out", str(out / "report")]) != 0:
        raise RuntimeError("sensitivity report failed")


def run_nightly(s: dict, args: argparse.Namespace, out: Path, config: str) -> tuple[bool, str]:
    if s["kind"] == "montecarlo":
        imperfections = str(repo_path(s["imperfections"]))
        runs = ("--runs", str(s["runs"]), "--seed0", str(s["seed"]))
        bench(
            args,
            "montecarlo",
            "--config",
            config,
            "--imperfections",
            imperfections,
            *runs,
            "--out",
            str(out),
        )
    else:
        bench(
            args,
            "sweep",
            "--config",
            config,
            "--sweep",
            str(repo_path(s["sweep"])),
            "--out",
            str(out),
        )
    summary = sensitivity.load_summary(out / "summary.csv")
    if not args.no_report:
        sensitivity_report(s, out / "summary.csv", out)
    if s["kind"] == "sweep":
        produced = args.no_report or (out / "report/hardware_requirements.md").is_file()
        return produced, f"{len(summary)} runs, report {'produced' if produced else 'missing'}"
    rate = float(sensitivity.passes(summary, thresholds_of(s)).mean())
    limit = float(s["thresholds"]["pass_rate"])
    return rate >= limit, f"pass rate {rate:.4g} over {len(summary)} runs (limit {limit:g})"


def judge_run(s: dict, csv: Path, out: Path, report: bool) -> tuple[bool, str]:
    m = compute_metrics(load_run(csv), tau_max_of(s))
    failures = check(m, thresholds_of(s))
    if report:
        # Imported here: the arm64 image has no matplotlib and runs with --no-report.
        from cobot_eval import report as report_mod

        (out / "scenario.yaml").write_text(yaml.safe_dump(s), encoding="utf-8")
        report_mod.main(
            ["--csv", str(csv), "--thresholds", str(out / "scenario.yaml"), "--out", str(out)]
            + ["--urdf", str(urdf_of(s)), "--title", f"{s['id']} report"]
        )
    pos, rot = m.final_pos_err_m * 1e3, np.degrees(m.final_rot_err_rad)
    good = f"pos {pos:.3f} mm, rot {rot:.3f} deg, rms {m.rms_joint_err_rad:.4g} rad"
    return not failures, "; ".join(failures) or good


def judge_safety(s: dict, csv: Path) -> tuple[bool, str]:
    df = load_run(csv)
    want, tau = s["expect"]["mode_reached"], joint_array(df, "tau")
    problems = [] if (df["mode"].astype(str) == want).any() else [f"{want} never reached"]
    if not np.all(np.isfinite(tau)):
        problems.append("non-finite torque")
    elif np.any(np.abs(tau) > tau_max_of(s)):
        problems.append(f"|tau| / tau_max peaks at {np.max(np.abs(tau) / tau_max_of(s)):.4g}")
    return not problems, "; ".join(problems) or f"{want} reached, torque within limits"


def run_scenario(s: dict, args: argparse.Namespace) -> tuple[bool, str]:
    out = args.out / str(s["id"])
    out.mkdir(parents=True, exist_ok=True)
    config = str(repo_path(s["config"]))
    if s["kind"] == "timing":
        text = bench(args, "timing", "--config", config, "--steps", str(s["steps"])).stdout
        match = re.search(r"p95_us=(\S+)", text)
        if match is None:
            return False, f"no p95_us in output: {text.strip()}"
        limit = float(s["thresholds"]["p95_step_us"])
        return float(match.group(1)) <= limit, f"{text.strip()} (limit p95 {limit:g} us)"
    if s["kind"] in ("montecarlo", "sweep"):
        return run_nightly(s, args, out, config)
    csvs = [out / f"run{k}.csv" for k in range(1, 3 if s["kind"] == "determinism" else 2)]
    for csv in csvs:
        imperfections = str(repo_path(s["imperfections"]))
        seed_out = ("--seed", str(s["seed"]), "--out", str(csv))
        bench(args, "run", "--config", config, "--imperfections", imperfections, *seed_out)
    if s["kind"] == "determinism":
        same = csvs[0].read_bytes() == csvs[1].read_bytes()
        return same, "identical bytes" if same else "the two runs differ"
    if s["kind"] == "safety":
        return judge_safety(s, csvs[0])
    if s["kind"] == "run":
        return judge_run(s, csvs[0], out, not args.no_report)
    return False, f"unknown kind {s['kind']}"


def evaluate(s: dict, args: argparse.Namespace) -> str:
    if s.get("runner") == "gazebo":
        if args.tag != "gazebo":
            return "SKIP   gazebo runner (use --tag gazebo)"
        return "FAIL   gazebo runner not available yet"
    try:
        ok, detail = run_scenario(s, args)
    except (OSError, ValueError, KeyError, TypeError, RuntimeError, yaml.YAMLError) as err:
        ok, detail = False, f"error: {err}"
    return f"{'PASS' if ok else 'FAIL'}   {detail}"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="run_e2e", description=__doc__)
    which = parser.add_mutually_exclusive_group(required=True)
    which.add_argument("--scenario", type=Path)
    which.add_argument("--all", action="store_true")
    parser.add_argument("--tag", default=None)
    parser.add_argument("--bench", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--no-report", action="store_true")
    args = parser.parse_args(argv)
    args.bench, args.out = args.bench.resolve(), args.out.resolve()

    paths = [args.scenario] if args.scenario else sorted(SCENARIO_DIR.glob("*.yaml"))
    try:
        scenarios = [yaml.safe_load(p.read_text(encoding="utf-8")) for p in paths]
    except (OSError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 2
    scenarios = [
        s
        for s in scenarios
        if (args.tag in s.get("tags", []) if args.tag else "nightly" not in s.get("tags", []))
        or args.scenario
    ]
    if not scenarios:
        print("error: no scenario selected", file=sys.stderr)
        return 2
    rows = [(str(s.get("id")), str(s.get("kind")), evaluate(s, args)) for s in scenarios]
    print(f"{'scenario':<10} {'kind':<12} result")
    for sid, kind, result in rows:
        print(f"{sid:<10} {kind:<12} {result}")
    return 1 if any(r.startswith("FAIL") for _, _, r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
