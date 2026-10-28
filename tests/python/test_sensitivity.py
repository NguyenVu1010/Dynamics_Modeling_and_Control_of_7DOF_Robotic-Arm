"""cobot_eval.sensitivity on a synthetic Summary CSV (contract: docs/conventions.md).

CLI: `python -m cobot_eval.sensitivity --summary S --thresholds YAML --out DIR`, thresholds
read from a scenario file's `thresholds:` mapping (as cobot_eval.report). Exit 0 when the
report is written (failing levels are results, not errors), 2 on bad input.

Rules pinned here (design 11.2):
- A level passes when every seed meets every threshold (final_pos_err_m, final_rot_err_deg
  from the final_rot_err_rad column, rms_joint_err_rad; non-finite fails).
- Severity order of a param's levels: numeric order starting from the end that passes, i.e.
  ascending unless the smallest level fails and the largest passes (encoder bits: fewer bits is
  worse). Walking that order, "worst passing level" is the last level before the first failing
  one and "first failing level" is that failing level; a passing level after it does not count
  (non-monotone metric). Columns of hardware_requirements.md:
  `| param | worst passing level | first failing level | metric that fails |`, "-" when none.
- tornado(summary, metric) -> [(param, change)], largest change first; change = mean over seeds
  at the worst level minus mean at the baseline (level 0, or the best level when 0 is not
  swept). The data below makes level 0 / the best level the same row, so every reading of
  "from level 0 to worst level" gives the same numbers.
- montecarlo.md (only when param=none rows exist): pass rate and, per metric, a table row
  `| metric | p50 | p95 | ...` with numpy's default (linear) percentiles.
"""

import importlib
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
COLUMNS = [
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
]
THRESHOLDS = """\
id: E2E-02
thresholds:
  final_pos_err_m: 0.005
  final_rot_err_deg: 2.0
  rms_joint_err_rad: 0.02
"""
GOOD = {"final_pos_err_m": 0.001, "final_rot_err_rad": 0.01, "rms_joint_err_rad": 0.005}

DELAY = "actuator.command_delay_cycles"
ENCODER = "sensor.encoder_bits"
COULOMB = "friction.coulomb_nm"
JITTER = "timing.period_jitter_std_us"


def passes(m):
    # 0.04 rad = 2.29 deg > 2 deg fails; compared as rad against 2.0 it would wrongly pass.
    return (
        m["final_pos_err_m"] <= 0.005
        and math.degrees(m["final_rot_err_rad"]) <= 2.0
        and m["rms_joint_err_rad"] <= 0.02
    )


def sweep_rows():
    """(param, level, seed, metrics) for two seeds per level."""
    rows = []

    def add(param, levels, metric, seed1, seed2):
        for level, v1, v2 in zip(levels, seed1, seed2, strict=True):
            for seed, v in ((1, v1), (2, v2)):
                rows.append((param, level, seed, GOOD | {metric: v}))

    # Monotone, ascending: 0.004 / 0.0036 pass at 2 cycles, 0.006 / 0.0054 fail at 4 > 5 mm.
    pos = [0.001, 0.002, 0.004, 0.006, 0.012]
    add(DELAY, [0, 1, 2, 4, 8], "final_pos_err_m", pos, [0.9 * v for v in pos])
    # Monotone, descending (fewer bits is worse): 14 bits 0.015 / 0.0165 pass, 12 bits 0.03
    # / 0.033 fail > 0.02 rad.
    rms = [0.05, 0.03, 0.015, 0.008, 0.006, 0.005]
    add(ENCODER, [10, 12, 14, 16, 18, 20], "rms_joint_err_rad", rms, [1.1 * v for v in rms])
    # Non-monotone, and only one seed fails at 2 N m (0.04 rad = 2.29 deg > 2 deg); 4 N m passes
    # again (0.015 rad = 0.86 deg) and 8 N m fails: worst passing must be 1, not 4.
    add(
        COULOMB,
        [0, 1, 2, 4, 8],
        "final_rot_err_rad",
        [0.01, 0.012, 0.04, 0.015, 0.06],
        [0.01, 0.012, 0.02, 0.015, 0.06],
    )
    # Every level passes.
    jit = [0.001, 0.0012, 0.0015, 0.002]
    add(JITTER, [0, 20, 100, 300], "final_pos_err_m", jit, jit)
    return rows


def montecarlo_rows():
    """20 runs: final_pos_err k * 0.1 mm (k = 1..20, all pass); runs 19 and 20 have an RMS joint
    error of 0.03 > 0.02 rad, so 18/20 = 0.9 pass."""
    rows = []
    for k in range(1, 21):
        m = GOOD | {"final_pos_err_m": 1e-4 * k, "rms_joint_err_rad": 0.03 if k > 18 else 0.005}
        rows.append(("none", 0, 100 + k, m))
    return rows


def write_summary(path, rows, drop=None):
    cols = [c for c in COLUMNS if c != drop]
    lines = [",".join(cols)]
    for run_id, (param, level, seed, m) in enumerate(rows):
        values = {
            "run_id": run_id,
            "param": param,
            "level": level,
            "seed": seed,
            **m,
            "peak_tau_ratio": 0.5,
            "sat_time_s": 0.0,
            "held": 0,
            "passed": int(passes(m)),
        }
        lines.append(
            ",".join(
                repr(values[c]) if isinstance(values[c], float) else str(values[c]) for c in cols
            )
        )
    path.write_bytes(("\n".join(lines) + "\n").encode())
    return path


def sensitivity():
    # Imported per test so that each test fails on its own while the module does not exist.
    return importlib.import_module("cobot_eval.sensitivity")


def run_cli(tmp_path, summary):
    thresholds = tmp_path / "thresholds.yaml"
    thresholds.write_bytes(THRESHOLDS.encode())
    out = tmp_path / "out"
    env = dict(os.environ, MPLBACKEND="Agg")
    env["PYTHONPATH"] = os.pathsep.join([str(REPO / "eval"), env.get("PYTHONPATH", "")])
    cmd = [sys.executable, "-m", "cobot_eval.sensitivity", "--summary", str(summary)]
    cmd += ["--thresholds", str(thresholds), "--out", str(out)]
    proc = subprocess.run(cmd, cwd=tmp_path, env=env, capture_output=True, text=True, timeout=300)
    return proc, out


def table_rows(text):
    """Markdown table rows as lists of stripped cells."""
    rows = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("|") and line.endswith("|") and not set(line) <= set("|-: "):
            rows.append([c.strip() for c in line.strip("|").split("|")])
    return rows


def number(cell):
    try:
        return float(cell.rstrip("%"))
    except ValueError:
        return None


def normalized(name):
    return re.sub(r"[^a-z0-9]", "", name.lower())


@pytest.fixture
def sweep_report(tmp_path):
    summary = write_summary(tmp_path / "summary.csv", sweep_rows())
    proc, out = run_cli(tmp_path, summary)
    assert proc.returncode == 0, proc.stderr
    return out


def requirement(out, param):
    text = (out / "hardware_requirements.md").read_text(encoding="utf-8")
    (row,) = [r for r in table_rows(text) if r[0].strip("`") == param]
    return row


def test_requirement_monotone_ascending(sweep_report):
    _, worst, first, metric = requirement(sweep_report, DELAY)[:4]
    assert number(worst) == 2 and number(first) == 4
    assert metric == "final_pos_err_m"


def test_requirement_monotone_descending_encoder_bits(sweep_report):
    # "encoder >= 14 bit": walking from 20 bits down, 12 bits is the first that fails.
    _, worst, first, metric = requirement(sweep_report, ENCODER)[:4]
    assert number(worst) == 14 and number(first) == 12
    assert metric == "rms_joint_err_rad"


def test_requirement_non_monotone_stops_at_first_failure(sweep_report):
    _, worst, first, metric = requirement(sweep_report, COULOMB)[:4]
    assert number(worst) == 1 and number(first) == 2
    assert metric.startswith("final_rot_err")


def test_requirement_all_levels_pass(sweep_report):
    _, worst, first, metric = requirement(sweep_report, JITTER)[:4]
    assert number(worst) == 300
    assert first == "-" and metric == "-"


def test_requirement_no_level_passes(tmp_path):
    rows = [(DELAY, level, 1, GOOD | {"final_pos_err_m": 0.01}) for level in (0, 1, 2)]
    proc, out = run_cli(tmp_path, write_summary(tmp_path / "summary.csv", rows))
    assert proc.returncode == 0, proc.stderr
    _, worst, _, metric = requirement(out, DELAY)[:4]
    assert worst == "-"
    assert metric == "final_pos_err_m"


def test_plots_per_param_and_tornado(sweep_report):
    pngs = list(sweep_report.glob("*.png"))
    for png in pngs:
        assert png.read_bytes()[:8] == b"\x89PNG\r\n\x1a\n", png
    names = [normalized(p.stem) for p in pngs]
    for param in (DELAY, ENCODER, COULOMB, JITTER):
        assert any(normalized(param) in n for n in names), (param, names)
    assert any("tornado" in n for n in names), names
    # A sweep-only summary has no Monte Carlo rows.
    assert not (sweep_report / "montecarlo.md").exists()


def test_tornado_orders_by_change(tmp_path):
    summary = sensitivity().load_summary(write_summary(tmp_path / "summary.csv", sweep_rows()))
    pos = sensitivity().tornado(summary, "final_pos_err_m")
    assert {p for p, _ in pos} == {DELAY, ENCODER, COULOMB, JITTER}
    # Delay: mean at 8 = (0.012 + 0.0108) / 2 = 0.0114, at 0 = (0.001 + 0.0009) / 2 = 0.00095
    # -> 0.01045. Jitter: 0.002 - 0.001 = 0.001. Encoder and Coulomb keep 0.001 -> 0.
    assert [p for p, _ in pos[:2]] == [DELAY, JITTER]
    assert pos[0][1] == pytest.approx(0.01045, abs=1e-12)
    assert pos[1][1] == pytest.approx(0.001, abs=1e-12)
    assert [c for _, c in pos[2:]] == pytest.approx([0.0, 0.0], abs=1e-12)

    rms = sensitivity().tornado(summary, "rms_joint_err_rad")
    # Encoder: worst 10 bits (0.05 + 0.055) / 2 = 0.0525, best 20 bits (0.005 + 0.0055) / 2 =
    # 0.00525 -> 0.04725; every other param keeps 0.005 -> 0.
    assert rms[0][0] == ENCODER
    assert rms[0][1] == pytest.approx(0.04725, abs=1e-12)


def test_load_summary_rejects_missing_column(tmp_path):
    path = write_summary(tmp_path / "summary.csv", sweep_rows(), drop="held")
    with pytest.raises(ValueError, match="held"):
        sensitivity().load_summary(path)
    proc, _ = run_cli(tmp_path, path)
    assert proc.returncode == 2
    assert "held" in proc.stderr


def test_montecarlo_pass_rate_and_percentiles(tmp_path):
    proc, out = run_cli(tmp_path, write_summary(tmp_path / "summary.csv", montecarlo_rows()))
    assert proc.returncode == 0, proc.stderr
    text = (out / "montecarlo.md").read_text(encoding="utf-8")
    lines = [ln for ln in text.splitlines() if "pass rate" in ln.lower()]
    assert lines, text

    def shows_rate(line):
        values = [float(v) for v in re.findall(r"\d+(?:\.\d+)?", line)]
        return 0.9 in values or ("%" in line and 90.0 in values)

    assert any(shows_rate(ln) for ln in lines), lines

    stats = {
        r[0].strip("`"): [number(c) for c in r[1:] if number(c) is not None]
        for r in table_rows(text)
    }
    # final_pos_err_m sorted = 0.1 .. 2.0 mm. p50: index 0.5 * 19 = 9.5 -> (1.0 + 1.1) / 2 =
    # 1.05 mm. p95: index 0.95 * 19 = 18.05 -> 1.9 + 0.05 * 0.1 = 1.905 mm.
    assert stats["final_pos_err_m"][:2] == pytest.approx([0.00105, 0.001905], rel=1e-3)
    # rms_joint_err_rad sorted = 18 x 0.005, 2 x 0.03. p50 = 0.005; p95 at index 18.05 lies
    # between the two 0.03 values -> 0.03.
    assert stats["rms_joint_err_rad"][:2] == pytest.approx([0.005, 0.03], rel=1e-3)


def test_hardware_sweep_file_matches_design():
    spec = yaml.safe_load((REPO / "config/sweeps/hardware.yaml").read_text(encoding="utf-8"))
    assert spec["base_profile"] == "config/imperfections/ideal.yaml"
    assert spec["seeds"] == [1, 2, 3]
    # float(): PyYAML reads 1e-5 (no dot) as a string; yaml-cpp and the bench read a number.
    levels = {p["name"]: [float(v) for v in p["levels"]] for p in spec["params"]}
    assert levels == {
        "sensor.encoder_bits": [10, 12, 14, 16, 18, 20],
        "sensor.q_noise_std_rad": [0, 1e-5, 1e-4, 5e-4, 1e-3],
        "sensor.sensor_delay_cycles": [0, 1, 2, 4, 8],
        "actuator.command_delay_cycles": [0, 1, 2, 4, 8],
        "actuator.torque_scale_std": [0, 0.05, 0.1, 0.2, 0.3],
        "actuator.torque_ripple_frac": [0, 0.02, 0.05, 0.1],
        "actuator.torque_cmd_resolution_nm": [0, 0.1, 0.5, 1.0, 2.0],
        "actuator.motor_time_constant_s": [0, 0.001, 0.003, 0.01],
        "friction.coulomb_nm": [0, 1, 2, 4, 8],
        "model.link_mass_scale_std": [0, 0.05, 0.1, 0.2],
        "model.payload_kg": [0, 0.5, 1, 2, 3],
        "timing.period_jitter_std_us": [0, 20, 100, 300],
    }
    # Every swept name is a key of the base profile.
    base = yaml.safe_load((REPO / spec["base_profile"]).read_text(encoding="utf-8"))
    for name in levels:
        group, key = name.split(".")
        assert key in base["imperfections"][group], name
