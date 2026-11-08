"""Stand-in for `cobot_bench verify` with lists, used by test_run_e2e_compare.py (E2E-16/17).

FAKE_BENCH_LOG: file that gets one JSON line (argv) per call. --hardware and --algorithm are
comma-separated lists; the model name is the file's stem. For each pair the model lists (as the
reference models: premium_fr3 and mid_kr1205 list computed_torque and pd_gravity, budget_xarm7
also drive_position) writes <--out>/<model>__<algorithm>/report.json and report.md; with more
than one pair also <--out>/comparison.json (disclaimer first, then rows) and comparison.md.
Verdicts as #81 measured: premium and mid computed_torque pass, budget computed_torque fail with
the position loop suggestion, budget drive_position marginal; an unlisted pair "not listed".
FAKE_BENCH_MODE, comma-separated flags:
  budget_drive_fail  budget_xarm7__drive_position fails;
  no_suggestion      the budget computed_torque report has no suggestion;
  no_comparison      no comparison.json;
  crash              exit 3, no output.
Any other mode than `verify` exits 3.
"""

import json
import os
import sys
from pathlib import Path

DISCLAIMER = (
    "Screening and comparison tool, not a certification: it does not replace a safety "
    "assessment to ISO 10218 or ISO/TS 15066, nor measurements on the real robot."
)
POSITION_LOOP = "needs a position loop in the drive (algorithm drive_position)"
LISTS = {
    "premium_fr3": ("computed_torque", "pd_gravity"),
    "mid_kr1205": ("computed_torque", "pd_gravity"),
    "budget_xarm7": ("computed_torque", "pd_gravity", "drive_position"),
}
VERDICTS = {
    "premium_fr3__computed_torque": ("pass", 100),
    "mid_kr1205__computed_torque": ("pass", 100),
    "budget_xarm7__computed_torque": ("fail", 0),
    "budget_xarm7__drive_position": ("marginal", 92),
}


def flag(argv: list[str], name: str) -> str:
    return argv[argv.index(name) + 1]


def report(model: str, algorithm: str, runs: int, flags: set[str]) -> dict:
    pair = f"{model}__{algorithm}"
    verdict, passed = VERDICTS.get(pair, ("marginal", runs))
    if pair == "budget_xarm7__drive_position" and "budget_drive_fail" in flags:
        verdict, passed = "fail", 85
    suggestions = []
    if pair == "budget_xarm7__computed_torque" and "no_suggestion" not in flags:
        suggestions = [POSITION_LOOP]
    return {
        "schema": 1,
        "disclaimer": DISCLAIMER,
        "verdict": verdict,
        "quick": runs < 100,
        "runs": runs,
        "pass_count": passed,
        "suggestions": suggestions,
        "model": model,
        "algorithm": algorithm,
    }


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_BENCH_MODE", "").split(",")))
    log = Path(os.environ["FAKE_BENCH_LOG"])
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sys.argv[1:]) + "\n")
    argv = sys.argv[1:]
    if argv[:1] != ["verify"] or "crash" in flags:
        return 3
    out = Path(flag(argv, "--out"))
    runs = int(flag(argv, "--runs")) if "--runs" in argv else 100
    models = [Path(p).stem for p in flag(argv, "--hardware").split(",")]
    algorithms = flag(argv, "--algorithm").split(",")
    pairs = [(m, a) for m in models for a in algorithms]
    listed = [(m, a) for m, a in pairs if a in LISTS.get(m, ())]
    if not listed:
        return 1
    rows = []
    for m, a in pairs:
        if (m, a) not in listed:
            rows.append({"model": m, "algorithm": a, "verdict": "not listed"})
            continue
        data = report(m, a, runs, flags)
        target = out / f"{m}__{a}" if len(listed) > 1 else out
        target.mkdir(parents=True, exist_ok=True)
        (target / "report.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        (target / "report.md").write_text(DISCLAIMER + "\n", encoding="utf-8")
        rows.append({k: data[k] for k in ("model", "algorithm", "verdict", "pass_count", "runs")})
    if len(listed) > 1 and "no_comparison" not in flags:
        comparison = {"disclaimer": DISCLAIMER, "rows": rows}
        (out / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n", "utf-8")
        (out / "comparison.md").write_text(DISCLAIMER + "\n", encoding="utf-8")
    print(DISCLAIMER)
    return 0


if __name__ == "__main__":
    sys.exit(main())
