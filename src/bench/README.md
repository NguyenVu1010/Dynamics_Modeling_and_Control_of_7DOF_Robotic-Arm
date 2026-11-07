# bench

`cobot_bench`: runs the control loop against the internal simulator without ROS, for e2e tests,
timing (PF-01), Monte Carlo and sensitivity sweeps. CLI contract in `docs/conventions.md`.
The binary is added by the `bench-e2e` feature; the root CMake picks up `bench/CMakeLists.txt`
when it exists.

In-memory API (`run_data.hpp`, `metrics.hpp`): `run_in_memory` returns one run's records, meta and
`RunSummary` without files; `write_run_files` writes the bytes `cobot_bench run` writes from it.

Auto-tune (`tune.hpp`, ADR-023): `bench::tune` sweeps the law's one gain (`tuning_key`, `with_gain`)
over seeds through `run_with` (a run on a list of controller overrides) and picks the best admissible
level (`select_gain`); `cobot_bench tune` writes `tune.csv` (docs/conventions.md "Tune CSV"). The
runs go through `run_queue` (`work_queue.hpp`), a small pool of worker threads whose results are
stored by item index, so the result does not depend on scheduling. The run path from `run_with`
down (config and waypoint loaders, URDF parser, Recorder, SimPlant, imperfection RNGs, metrics)
holds no `static` or global mutable state: every run owns its objects.

Timing: `cobot_bench timing --config <yaml> --steps <n>` prints the PF-01 line
`p50_us=... p95_us=... max_us=...`. `--paced` steps at the control rate with `clock_nanosleep`
(absolute deadlines) and `--log-mode off|recorder|recorder_live` overrides `cobot.log.mode`; with
either flag the line is PF-02's (ADR-022): `p50_us p95_us p99_us max_us log_mode paced late_cycles
late_p99_us reader_samples reader_lost reader_cpu`. `recorder_live` publishes every step into the
studio bridge region `/cobot_timing_<pid>` (removed at the end) while a reader thread drains it.
With at least two allowed CPUs every paced run pins the loop thread to the first one, whatever the
log mode, so the three modes compare; the reader goes to the last one.
