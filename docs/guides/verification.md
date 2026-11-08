# Hardware verification

"Is this hardware good enough for this task with this control law?" `cobot_bench verify` and the
Verification tab of `cobot_studio` answer it with a verdict, an interval and the assumptions it
rests on ([ADR-023](../adr/023-hardware-scenario-verification.md)). Results of the reference
models: [docs/results/verify](../results/verify/README.md).

> Screening and comparison tool, not a certification: it does not replace a safety assessment to
> ISO 10218 or ISO/TS 15066, nor measurements on the real robot. Every report starts with this
> sentence.

## What a verification does

1. **Tune.** Sweeps the law's one gain (`computed_torque` and `pd_gravity`: ω 20..150; drive
   position loop: ω 20..200 rad/s) over tuning seeds 1..10 and keeps the level with the lowest
   median tracking error among those with no safety hold, torque saturation under 5 % of the run
   and a post-goal swing that decays. `--no-tune` keeps the model's own gain. If no level
   qualifies, the verdict is on the model's gain and the report says so.
2. **Monte Carlo.** 100 runs (seeds 1..100 by default) of the task at the gain used. A run passes
   when it does not hold and every requirement of the task is met: final position and rotation
   error, RMS joint error, and cycle time (the time until the TCP enters and stays within the
   final thresholds).
3. **Verdict** from the 95 % Wilson interval of the pass rate, against the task's target (default
   95 %) and margin (default 20 %):

   | Verdict | When |
   | --- | --- |
   | PASS | lower bound ≥ target, every worst value ≤ (1 − margin) × threshold, hold rate within the task's limit; never in quick mode |
   | FAIL | upper bound < target, or the hold rate is provably above the task's limit |
   | MARGINAL, measure on hardware | otherwise |

   With 100 runs, PASS needs 100/100 and FAIL means 90/100 or fewer. Fewer than 100 runs, or
   `--quick`, can give MARGINAL or FAIL only.
4. **Sensitivity.** Each `assumed` or `derived` parameter of the model is run at the low and at the
   high end of its bounds (20 seeds each by default). The report marks it "may change" when either
   end lands on the other side of the target from the main pass rate, or holds appear. Off in quick
   mode and with `--no-sensitivity`.
5. **On FAIL: cause and suggestion.** Each imperfection group is switched off in turn (20 seeds);
   the group whose removal raises the pass count most is the main cause. Suggestions come from a
   fixed rule table (for example "needs a position loop in the drive").

The result does not depend on `--jobs`: the same inputs give the same report bytes.

## Command line

```bash
cobot_bench verify --hardware config/hardware/budget_xarm7.yaml \
  --algorithm computed_torque,drive_position --task config/tasks/reference_move.yaml --out out/v
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--hardware a.yaml[,b.yaml...]` | required | hardware model files |
| `--algorithm x[,y...]` | required | `computed_torque`, `pd_gravity`, `drive_position` |
| `--task t.yaml` | required | task file |
| `--runs n`, `--seed0 n` | 100, 1 | Monte Carlo runs and first seed |
| `--quick` | off | caps the verdict at MARGINAL |
| `--no-tune`, `--levels a,b`, `--tune-seeds n` | tune on, default levels, 10 | tuning step |
| `--no-sensitivity`, `--sensitivity-seeds n` | on, 20 | sensitivity step |
| `--ablation-seeds n` | 20 | cause analysis on FAIL |
| `--jobs n` | number of cores (at most 8) | worker threads |
| `--out dir` | required | output directory |

Outputs: `report.md` and `report.json` (same content; schema in
[Verify report](../conventions.md#verify-report)) and `tune.csv`. With several models or
algorithms, each pair gets `<out>/<model>__<algorithm>/` and `<out>/comparison.{md,json}` has one
row per pair; a pair the model does not list is shown as "not listed". Exit 0 for every verdict, 1
on bad input (nothing written).

Run time grows with tuning (levels × tuning seeds), runs and sensitivity (2 × sensitivity seeds per
assumed key). On a 4-core CI runner the four-pair comparison of the results page took 2 h 41 min
with 10 sensitivity seeds, and 34 min without sensitivity; a quick run of one pair
(`--quick --runs 10 --levels 40,50 --tune-seeds 2`) took 14 s on a desktop.

## Studio

The Verification tab ([studio guide](studio.md#verification-tab)) takes the same choices, runs the
job on a worker thread with progress and cancel, and shows the verdict, interval, worst values,
assumptions, tuning table, causes and the comparison. It writes its reports through the same
library function as the command line, so both give the same files.

## Adding a hardware model

1. Write the imperfection profile `config/imperfections/<name>.yaml` and a controller file
   ([configuration](configuration.md)).
2. Copy `config/hardware/premium_fr3.yaml` to `config/hardware/<name>.yaml`, and point
   `imperfections` and `controller` at your files. Set `limits`, and list the algorithms the
   hardware supports with their starting gains.
3. Give every active imperfection key a `provenance` entry: `published` (a datasheet), `derived`
   or `assumed`. The last two need bounds: `range: [lo, hi]` for a scalar or `scale: [lo, hi]` for
   a per-joint list. The loader refuses a model with an unsourced key and names it, so loading the
   file tells you what is missing. These bounds drive the sensitivity step: they should say how
   wrong the value could be.
4. Run `cobot_bench verify --quick --runs 10` first, then the full 100 runs.

Task files (`config/tasks/`) hold the trajectory or waypoint list, the payload and the
requirements. Schemas: [Hardware model file](../conventions.md#hardware-model-file),
[Task file](../conventions.md#task-file). The thresholds of the two reference tasks are proposals
awaiting the owner's approval.
