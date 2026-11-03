"""Stand-in for `cobot_bench montecarlo` used by test_run_e2e_profiles.py (kind profiles, E2E-12).

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. The profile is the stem of
--imperfections. Clean rows: tip 1 mm from the goal, 0.001 rad, rms 0.002 rad, no hold, every
row flagged passed=1 on purpose (the runner judges from the metric columns). budget_xarm7: the
first run (seed0) holds with state_timeout at 0.006 s and ends 0.87 m from the goal (a hold
early in the run), the others are clean, so its hold rate is 1/3 with runs 3.
FAKE_BENCH_MODE, comma-separated flags:
  mid_fail          mid_kr1205 run 1 ends 10 mm from the goal;
  premium_hold      premium_fr3 run 0 holds with state_timeout;
  budget_overspeed  budget_xarm7 run 1 holds with overspeed;
  budget_all_held   every budget_xarm7 run holds with state_timeout;
  crash             exit 3, no output.
`run` and `timing` exit 3 (this fake only serves montecarlo).
"""

import argparse
import json
import os
import sys
from pathlib import Path

COLUMNS = (
    "run_id,param,level,seed,final_pos_err_m,final_rot_err_rad,rms_joint_err_rad,"
    "peak_tau_ratio,sat_time_s,rms_tau_ratio,held,hold_time_s,hold_reason,passed"
)


def row(run_id: int, seed: int, pos_err: float = 0.001, hold: str = "") -> str:
    held = f"1,0.006,{hold}" if hold else "0,,none"
    return f"{run_id:04d},none,0,{seed},{pos_err},0.001,0.002,0.5,0,0.3,{held},1"


def rows_for(profile: str, runs: int, seed0: int, flags: set[str]) -> list[str]:
    out = []
    for k in range(runs):
        pos, hold = 0.001, ""
        if profile == "mid_kr1205" and "mid_fail" in flags and k == 1:
            pos = 0.01
        if profile == "premium_fr3" and "premium_hold" in flags and k == 0:
            hold = "state_timeout"
        if profile == "budget_xarm7":
            if k == 0 or "budget_all_held" in flags:
                pos, hold = 0.87, "state_timeout"
            if "budget_overspeed" in flags and k == 1:
                pos, hold = 0.87, "overspeed"
        out.append(row(k, seed0 + k, pos, hold))
    return out


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_BENCH_MODE", "").split(",")))
    log = Path(os.environ["FAKE_BENCH_LOG"])
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sys.argv[1:]) + "\n")
    if sys.argv[1:2] != ["montecarlo"] or "crash" in flags:
        return 3
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    mc = sub.add_parser("montecarlo")
    for name in ("--config", "--imperfections", "--seed0", "--out"):
        mc.add_argument(name, required=True)
    mc.add_argument("--runs", required=True, type=int)
    mc.add_argument("--keep-runs", action="store_true")
    args = parser.parse_args()
    for path in (args.config, args.imperfections):
        if not Path(path).is_file():
            print(f"{path}: no such file", file=sys.stderr)
            return 2
    rows = rows_for(Path(args.imperfections).stem, args.runs, int(args.seed0), flags)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "summary.csv").write_text("\n".join([COLUMNS, *rows]) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
