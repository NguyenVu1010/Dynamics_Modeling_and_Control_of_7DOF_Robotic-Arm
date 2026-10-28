"""Stand-in for cobot_bench montecarlo/sweep (roadmap CLI) used by test_run_e2e_sweep.py.

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. FAKE_BENCH_MODE: comma-separated
flags, empty = clean: mc_fail (two Monte Carlo runs over the 5 mm limit instead of one), crash
(exit 3, no output). Every summary row says passed=1 on purpose: thresholds live in the
scenario, so the runner judges from the metric columns, never from the bench's own flag.
`run` and `timing` exit 3 (this fake only serves the nightly modes).
"""

import argparse
import json
import os
import sys
from pathlib import Path

import yaml

COLUMNS = (
    "run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,"
    "peak_tau_ratio,sat_time_s,held,passed"
)


def row(run_id: int, param: str, level: object, seed: int, pos_err: float) -> str:
    return f"{run_id},{param},{level},{seed},{pos_err},0.001,0.002,0.5,0,0,1"


def montecarlo(args: argparse.Namespace, flags: set[str]) -> list[str]:
    failing = 2 if "mc_fail" in flags else 1
    seed0 = int(args.seed0)
    # The first `failing` runs end 10 mm from the goal; the others 1 mm.
    return [row(k, "none", 0, seed0 + k, 0.01 if k < failing else 0.001) for k in range(args.runs)]


def sweep(args: argparse.Namespace) -> list[str]:
    spec = yaml.safe_load(Path(args.sweep).read_text(encoding="utf-8"))
    rows = []
    for param in spec["params"]:
        for i, level in enumerate(param["levels"]):
            for seed in spec["seeds"]:
                rows.append(row(len(rows), param["name"], level, seed, 0.001 * (i + 1)))
    return rows


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_BENCH_MODE", "").split(",")))
    log = Path(os.environ["FAKE_BENCH_LOG"])
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sys.argv[1:]) + "\n")
    if sys.argv[1:2] in (["run"], ["timing"]) or "crash" in flags:
        return 3
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    mc = sub.add_parser("montecarlo")
    for name in ("--config", "--imperfections", "--seed0", "--out"):
        mc.add_argument(name, required=True)
    mc.add_argument("--runs", required=True, type=int)
    mc.add_argument("--keep-runs", action="store_true")
    sw = sub.add_parser("sweep")
    for name in ("--config", "--sweep", "--out"):
        sw.add_argument(name, required=True)
    args = parser.parse_args()
    if not Path(args.config).is_file():
        print(f"{args.config}: no such file", file=sys.stderr)
        return 2
    rows = montecarlo(args, flags) if args.cmd == "montecarlo" else sweep(args)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.csv").write_text("\n".join([COLUMNS, *rows]) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
