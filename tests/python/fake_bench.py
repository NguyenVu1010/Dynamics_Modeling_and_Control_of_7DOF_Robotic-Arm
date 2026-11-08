"""Stand-in for cobot_bench (roadmap CLI) used by test_run_e2e.py; stdlib only.

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. FAKE_BENCH_MODE: comma-separated
flags, empty = clean run (tracking, then finished): hold (add holding rows; also on when the
config file name contains "safety"), pos4mm / fail (final tip error 4 mm / 10 mm), nondet
(step_us differs between calls), tau_over (|tau1| = 250 > 204 (peak P of joint_1)), nan_tau,
slow (timing p95_us = 1500), crash (exit 3, no output), stop6mm (with --waypoints: the stop
waypoints lie 6 mm from the tip).
With --waypoints FILE (exit 2 when FILE is missing) the meta file gets the waypoint keys of
`cobot_bench run --waypoints`: four nodes at 10..40 ms, waypoints 1 and 3 stop, every position at
the fake tip (0.4, 0.1, 0.5), so stop_pos_err_m equals the final tip error.
`verify` (E2E-15 under --all --tag pr) writes <--out>/report.json and report.md: every key of the
Verify report, verdict marginal, quick true, the --hardware and --task files with their sha256.
"""

import argparse
import hashlib
import json
import os
import sys
from pathlib import Path


def write_run(args: argparse.Namespace, flags: set[str], calls: int) -> None:
    joints = [f"{p}{j}" for p in ("q", "qd", "qref", "qdref", "tau") for j in range(1, 8)]
    cols = ["t_ns", "mode", *joints]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    lines = [",".join(cols + [f"e{k}" for k in range(1, 7)] + ["step_us", "hold"])]
    hold = "hold" in flags or "safety" in Path(args.config).name
    err = 0.01 if "fail" in flags else 0.004 if "pos4mm" in flags else 0.0
    for k in range(50):
        mode = "holding" if hold and k >= 30 else "finished" if k >= 40 else "tracking"
        tau = [1.0] * 7
        if k == 35 and "tau_over" in flags:
            tau[0] = 250.0
        if k == 35 and "nan_tau" in flags:
            tau[1] = float("nan")
        q = [0.1 * j for j in range(7)]
        vals = [k * 1_000_000, mode, *q, *[0.0] * 7, *q, *[0.0] * 7, *tau, 0.4 + err, 0.1, 0.5]
        vals += [0.0] * 3 + [0.4, 0.1, 0.5] + [0.0] * 9 + [calls if "nondet" in flags else 0]
        vals.append("joint_limit" if mode == "holding" else "none")
        lines.append(",".join(str(v) for v in vals))
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    meta = f"config: {args.config}\nimperfections: {args.imperfections}\nseed: {args.seed}\n"
    if getattr(args, "waypoints", None):
        x = 0.406 if "stop6mm" in flags else 0.4
        meta += f"waypoints_file: {args.waypoints}\n"
        meta += "waypoint_node_times_s: [0.01, 0.02, 0.03, 0.04]\n"
        meta += "waypoint_stop: [false, true, false, true]\n"
        meta += f"waypoint_positions_m: [[0.4, 0.1, 0.5], [{x}, 0.1, 0.5], [0.4, 0.1, 0.5], "
        meta += f"[{x}, 0.1, 0.5]]\n"
    Path(args.out + ".meta.yaml").write_text(meta, encoding="utf-8")


def write_verify(args: argparse.Namespace) -> None:
    inputs = [
        {"path": p, "sha256": hashlib.sha256(Path(p).read_bytes()).hexdigest()}
        for p in (args.hardware, args.task)
    ]
    limits = {"value": 1e-4, "threshold": 0.005, "margin_limit": 0.004}
    names = ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s")
    tuning = {"key": "config.computed_torque.omega", "gain_used": 50.0, "gain_source": "tuned"}
    tuning |= {"levels": [], "rows": [], "chosen": 50.0, "reason": "fake"}
    report = {"schema": 1, "disclaimer": "fake", "verdict": "marginal", "quick": True}
    report |= {"runs": int(args.runs), "seeds": [1, int(args.runs)], "pass_count": int(args.runs)}
    report |= {"pass_rate": 1.0, "wilson": [0.7, 1.0], "hold_rate": 0.0}
    report |= {"worst": {n: dict(limits) for n in names}, "tuning": tuning, "assumptions": []}
    report |= {"causes": [], "suggestions": [], "versions": {"git": "fake", "inputs": inputs}}
    report |= {"model": "fake", "algorithm": args.algorithm, "task": "fake"}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.json").write_text(json.dumps(report) + "\n", encoding="utf-8")
    (out / "report.md").write_text("fake\n", encoding="utf-8")


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_BENCH_MODE", "").split(",")))
    log = Path(os.environ["FAKE_BENCH_LOG"])
    calls = len(log.read_text().splitlines()) if log.exists() else 0
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sys.argv[1:]) + "\n")
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run")
    for name in ("--config", "--imperfections", "--seed", "--out"):
        run.add_argument(name, required=True)
    run.add_argument("--duration-s")
    run.add_argument("--waypoints")
    timing = sub.add_parser("timing")
    timing.add_argument("--config", required=True)
    timing.add_argument("--steps", required=True, type=int)
    verify = sub.add_parser("verify")
    for name in ("--hardware", "--algorithm", "--task", "--runs", "--out"):
        verify.add_argument(name, required=True)
    verify.add_argument("--quick", action="store_true")
    verify.add_argument("--levels")
    verify.add_argument("--tune-seeds")
    args = parser.parse_args()
    if args.cmd == "verify":
        if "crash" in flags:
            return 3
        write_verify(args)
        return 0
    if not Path(args.config).is_file():
        print(f"{args.config}: no such file", file=sys.stderr)
        return 2
    if getattr(args, "waypoints", None) and not Path(args.waypoints).is_file():
        print(f"{args.waypoints}: no such file", file=sys.stderr)
        return 2
    if "crash" in flags:
        return 3
    if args.cmd == "timing":
        print(f"p50_us=120.5 p95_us={1500.0 if 'slow' in flags else 250.25} max_us=1900.75")
    else:
        write_run(args, flags, calls)
    return 0


if __name__ == "__main__":
    sys.exit(main())
