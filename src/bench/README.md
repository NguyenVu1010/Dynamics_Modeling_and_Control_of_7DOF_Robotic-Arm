# bench

`cobot_bench`: runs the control loop against the internal simulator without ROS, for e2e tests,
timing (PF-01), Monte Carlo and sensitivity sweeps. CLI contract in `docs/conventions.md`.
The binary is added by the `bench-e2e` feature; the root CMake picks up `bench/CMakeLists.txt`
when it exists.

In-memory API (`run_data.hpp`, `metrics.hpp`): `run_in_memory` returns one run's records, meta and
`RunSummary` without files; `write_run_files` writes the bytes `cobot_bench run` writes from it.
