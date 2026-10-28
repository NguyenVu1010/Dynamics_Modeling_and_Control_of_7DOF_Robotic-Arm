"""Stand-in for cobot_bench (roadmap CLI) used by test_run_e2e.py; stdlib only.

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. FAKE_BENCH_MODE: comma-separated
flags, empty = clean run (tracking, then finished): hold (add holding rows; also on when the
config file name contains "safety"), pos4mm / fail (final tip error 4 mm / 10 mm), nondet
(step_us differs between calls), tau_over (|tau1| = 100 > 87), nan_tau, slow (timing
p95_us = 1500), crash (exit 3, no output).
"""

import argparse
import json
import os
import sys
from pathlib import Path


def write_run(args: argparse.Namespace, flags: set[str], calls: int) -> None:
    joints = [f"{p}{j}" for p in ("q", "qd", "qref", "qdref", "tau") for j in range(1, 8)]
    cols = ["t_ns", "mode", *joints]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    lines = [",".join(cols + [f"e{k}" for k in range(1, 7)] + ["step_us"])]
    hold = "hold" in flags or "safety" in Path(args.config).name
    err = 0.01 if "fail" in flags else 0.004 if "pos4mm" in flags else 0.0
    for k in range(50):
        mode = "holding" if hold and k >= 30 else "finished" if k >= 40 else "tracking"
        tau = [1.0] * 7
        if k == 35 and "tau_over" in flags:
            tau[0] = 100.0
        if k == 35 and "nan_tau" in flags:
            tau[1] = float("nan")
        q = [0.1 * j for j in range(7)]
        vals = [k * 1_000_000, mode, *q, *[0.0] * 7, *q, *[0.0] * 7, *tau, 0.4 + err, 0.1, 0.5]
        vals += [0.0] * 3 + [0.4, 0.1, 0.5] + [0.0] * 9 + [calls if "nondet" in flags else 0]
        lines.append(",".join(str(v) for v in vals))
    Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    meta = f"config: {args.config}\nimperfections: {args.imperfections}\nseed: {args.seed}\n"
    Path(args.out + ".meta.yaml").write_text(meta, encoding="utf-8")


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
    timing = sub.add_parser("timing")
    timing.add_argument("--config", required=True)
    timing.add_argument("--steps", required=True, type=int)
    args = parser.parse_args()
    if not Path(args.config).is_file():
        print(f"{args.config}: no such file", file=sys.stderr)
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
