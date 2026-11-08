"""Stand-in for `cobot_bench verify` used by test_run_e2e_verify.py (kind verify, E2E-15).

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. Writes <--out>/report.json with every
key of the Verify report contract (docs/conventions.md), verdict "marginal", quick true, and in
versions.inputs the --hardware and --task files with the sha256 of their bytes, plus report.md.
FAKE_BENCH_MODE, comma-separated flags:
  verdict_pass   verdict "pass";
  not_quick      quick false;
  bad_hash       the first input's sha256 is wrong;
  missing_wilson no "wilson" key;
  no_report      exit 0 without writing report.json;
  crash          exit 3, no output.
Any other mode than `verify` exits 3.
"""

import hashlib
import json
import os
import sys
from pathlib import Path

DISCLAIMER = (
    "Screening and comparison tool, not a certification: it does not replace a safety "
    "assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot."
)


def flag(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def report(argv: list[str], flags: set[str]) -> dict:
    runs = int(flag(argv, "--runs"))
    inputs = []
    for name in ("--hardware", "--task"):
        path = Path(flag(argv, name))
        inputs.append({"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    if "bad_hash" in flags:
        inputs[0]["sha256"] = "0" * 64
    worst = {
        name: {"value": 1e-4, "threshold": 0.005, "margin_limit": 0.004}
        for name in ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "cycle_time_s")
    }
    data = {
        "schema": 1,
        "disclaimer": DISCLAIMER,
        "verdict": "pass" if "verdict_pass" in flags else "marginal",
        "quick": "not_quick" not in flags,
        "runs": runs,
        "seeds": [1, runs],
        "pass_count": runs,
        "pass_rate": 1.0,
        "wilson": [0.72, 1.0],
        "hold_rate": 0.0,
        "worst": worst,
        "tuning": {
            "key": "config.computed_torque.omega",
            "gain_used": 50.0,
            "gain_source": "tuned",
            "levels": [40.0, 50.0],
            "rows": [],
            "chosen": 50.0,
            "reason": "chosen 50",
        },
        "assumptions": [],
        "causes": [],
        "suggestions": [],
        "versions": {"git": "fake", "inputs": inputs},
        "model": "premium_fr3",
        "algorithm": flag(argv, "--algorithm"),
        "task": "reference_move",
    }
    if "missing_wilson" in flags:
        del data["wilson"]
    return data


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_BENCH_MODE", "").split(",")))
    log = Path(os.environ["FAKE_BENCH_LOG"])
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sys.argv[1:]) + "\n")
    argv = sys.argv[1:]
    if argv[:1] != ["verify"] or "crash" in flags:
        return 3
    out = Path(flag(argv, "--out"))
    out.mkdir(parents=True, exist_ok=True)
    if "no_report" in flags:
        return 0
    data = report(argv, flags)
    (out / "report.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    (out / "report.md").write_text(DISCLAIMER + "\n\nMARGINAL, measure on hardware\n", "utf-8")
    print(DISCLAIMER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
