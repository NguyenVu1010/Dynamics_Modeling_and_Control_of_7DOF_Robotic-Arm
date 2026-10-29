"""Severity order when every level passes (or every level fails).

The pass/fail pattern alone cannot tell which end is worse, so the level with the larger error
(sum of checked metric / threshold ratios, mean over seeds, more than 5 % larger) is the worse
end; otherwise the walk stays ascending. Without this, a param where more is better (encoder
bits) reported its best level as the "worst passing" one: 20 bits instead of 10.
"""

from test_sensitivity import ENCODER, GOOD, JITTER, requirement, run_cli, write_summary


def report(tmp_path, rows):
    proc, out = run_cli(tmp_path, write_summary(tmp_path / "summary.csv", rows))
    assert proc.returncode == 0, proc.stderr
    return out


def rows_for(param, levels, metric, values):
    return [
        (param, level, seed, GOOD | {metric: v})
        for level, v in zip(levels, values, strict=True)
        for seed in (1, 2)
    ]


def test_all_pass_descending_param_reports_its_smallest_level(tmp_path):
    # Fewer encoder bits, larger error; every level within 0.02 rad.
    rms = [0.0024, 0.0005, 0.00017, 0.00012]
    out = report(tmp_path, rows_for(ENCODER, [10, 12, 14, 20], "rms_joint_err_rad", rms))
    _, worst, first, metric = requirement(out, ENCODER)[:4]
    assert float(worst) == 10
    assert first == "-" and metric == "-"


def test_all_pass_ascending_param_reports_its_largest_level(tmp_path):
    pos = [0.001, 0.0012, 0.0015, 0.002]
    out = report(tmp_path, rows_for(JITTER, [0, 20, 100, 300], "final_pos_err_m", pos))
    assert float(requirement(out, JITTER)[1]) == 300


def test_all_pass_equal_errors_keep_ascending_order(tmp_path):
    out = report(tmp_path, rows_for(JITTER, [0, 20, 100], "final_pos_err_m", [0.001] * 3))
    assert float(requirement(out, JITTER)[1]) == 100


def test_all_fail_descending_param_names_its_best_level_as_first_failure(tmp_path):
    # Every level fails; the least bad end (20 bits) is where the walk starts.
    rms = [0.09, 0.06, 0.03]
    out = report(tmp_path, rows_for(ENCODER, [10, 14, 20], "rms_joint_err_rad", rms))
    _, worst, first, metric = requirement(out, ENCODER)[:4]
    assert worst == "-"
    assert float(first) == 20
    assert metric == "rms_joint_err_rad"


def test_all_pass_negligible_difference_keeps_ascending_order(tmp_path):
    # Motor time constant in the realism sweep: the error moves by ~1e-9 relative, which is
    # noise, not a direction. Only a clear difference (> 5 %) reverses the walk.
    pos = [0.0010000, 0.0009999, 0.0009998]
    out = report(tmp_path, rows_for(JITTER, [0, 0.001, 0.01], "final_pos_err_m", pos))
    assert float(requirement(out, JITTER)[1]) == 0.01
