# Contributing

Dynamics modelling and computed-torque control of a 7-DOF arm, written in C++17. The core
runs at 1 kHz on a desktop (Gazebo Harmonic, ROS 2 Jazzy, Ubuntu 24.04) and is meant to run
later on a Raspberry Pi 5.

These rules apply to every change. Architecture decisions live in `docs/adr/`. When a change
goes against an ADR, write a new ADR first. The design behind all of this is
`docs/design/2026-10-28-cpp-rework-design.md`.

## Repository layout

```
src/
  core/        C++17 library, no ROS. One CMake target per module:
    common/      fixed-size types (VecN, MatN, Pose, Timestamp), Result/Status, logger
                 interface, so3_exp/so3_log/pose_error
    model/       Model<N>: URDF parser (tinyxml2) -> flat joint chain, limits, inertia
    kinematics/  FK, geometric Jacobian 6xN, Jdot*qdot
    dynamics/    RNEA, CRBA, G, C*qdot, forward dynamics, SimPlant (RK4)
    planning/    quintic Cartesian trajectory, second-order CLIK reference generator
    control/     joint-space computed torque, safe hold controller
    hal/         JointInterface<N>, JointState<N>, JointCommand<N>
    imperfect/   configurable sensor/actuator/model imperfections (decorator on hal)
    pipeline/    ControlLoop, Config from YAML (only place that parses YAML), SPSC ring,
                 CSV writer, SimJointInterface
  ros2_node/   ament package: ros2_control ControllerInterface wrapping ControlLoop
  bench/       cobot_bench binary without ROS: ControlLoop + SimPlant, run/montecarlo/sweep/timing
  sim/         ament package cobot_sim: URDF/xacro, meshes, world SDF, controller config, launch
  eval/        Python (cobot_eval): metrics, reports, sensitivity analysis; never imports core
config/      default.yaml, imperfections/<profile>.yaml, sweeps/<name>.yaml
docs/        conventions.md, derivations/, adr/, design/, results/
docker/      pi5.Dockerfile (arm64, apt-only core deps), ros.Dockerfile (Jazzy + Gazebo)
tests/       unit/<module>, component/ (known-answer problems), e2e/scenarios/, python/,
             data/ (small fixtures)
tools/       wsl/ (setup, build, pytest, lint, sim), ci/, fix_urdf_inertia.py, run_e2e.py
```

## Dependency direction

Fixed, and enforced by the linker because every module is its own target `cobot::<module>`
(real name `cobot_<module>`, `INTERFACE` until it has sources):

```
pipeline -> control, planning, hal, imperfect
control  -> dynamics, hal
planning -> kinematics
imperfect -> hal, dynamics
dynamics -> kinematics -> model -> common
hal      -> common
```

- `ros2_node` and `bench` link only `cobot::pipeline`.
- `src/core/` never includes ROS, Gazebo, KDL or Pinocchio headers.
- External dependencies of `src/core/`: Eigen 3.4 (all), tinyxml2 (`model`), yaml-cpp
  (`pipeline`). Tests add GTest. Everything must be installable with `apt` on Ubuntu 24.04
  and Raspberry Pi OS. Adding anything else needs an ADR.
- KDL is allowed only in `tools/gen_reference_dynamics.cpp`, built with
  `-DCOBOT_WITH_KDL_REF=ON`, to produce the committed test fixture.
- `src/eval/` never links against or imports core. It reads only the Run CSV and Summary CSV
  defined in `docs/conventions.md`.
- CMake options and variables are prefixed `COBOT_`.

## Math and frame conventions (read docs/conventions.md)

- Frames: `b` base_link, `i` link i, `e` end effector (origin of `link_7` after merging
  fixed joints).
- `T_a_b` maps points from `b` to `a`: `p_a = T_a_b * p_b`. Composition must cancel:
  `T_b_e = T_b_i * T_i_e`. Rotations `R_a_b`, positions `p_a_b`.
- Twists and Jacobian rows are ordered `[v; ω]` and expressed in `b`. The geometric Jacobian
  is named `J_e_b`.
- Orientation error is `so3_log(R_d * R^T)`. No Euler angles inside the control loop.
- SI units, radians in code. Degrees only in YAML with a `_deg` suffix.
- Quaternions only at I/O boundaries, Hamilton order `(w, x, y, z)`, written in a comment
  wherever one is built.
- Time is `Timestamp` (int64 nanoseconds); `dt` is double seconds. Never compare
  floating-point times.
- Gravity is `[0, 0, -9.80665]` in `b`.

## C++ rules

- C++17, GCC 13 (desktop) and GCC 12+ (Pi OS). Warnings `-Wall -Wextra -Wpedantic`;
  `-Werror` through `COBOT_WERROR=ON` (CI and `tools/wsl/build.sh`).
- Formatting comes from `.clang-format` (clang-format 18), static checks from `.clang-tidy`
  (clang-tidy 18). Run `tools/wsl/lint.sh` before committing (pre-commit does the same).
- Naming: types `PascalCase`; functions, methods and variables `snake_case`; members get a
  trailing underscore (`model_`); constants `kPascalCase`; namespaces `cobot::<module>`.
- Headers use `#pragma once`. Include order: own header, C/C++ std, third party, project.
- The core is templated on the number of joints `N`; the project uses `Arm7 = Model<7>`.
  Use `VecN<N>`, `MatN<N>`, `Mat6N<N>` from `cobot/common/types.hpp`, never dynamic-size
  Eigen types in core.
- Do not use `auto` for Eigen expressions. Assign them to a concrete type so they are
  evaluated. Pass Eigen objects by `const&`.
- No magic numbers. Every tunable value lives in `config/default.yaml` (or an imperfection
  or sweep file), has a unit in its name or comment, and is range-checked at load time with
  an error message that names the full key. Unknown keys are errors.
- Errors:
  - Construction and config loading may throw.
  - `step()` and per-cycle functions return `Status` / `Result<T>`.
  - Never `abort` on bad data from the HAL; go to the hold mode and report.
- Ownership: prefer values and `std::unique_ptr`. Use `std::shared_ptr` only for data
  threads truly share, and document which.
- The core never creates threads. The host (`bench` or the `ros2_control` plugin) owns the
  control thread and the CSV writer thread; they exchange data only through the fixed-size
  `pipeline::SpscRing`.
- Logging goes through the core logger interface; `ros2_node` adapts it to rclcpp. No
  `std::cout` in core.
- Interface headers under `src/core/<module>/include/cobot/<module>/*.hpp` are the contract
  between test and implementation work. Changing a signature needs the owner's agreement
  and a note in the PR.

## Hot path rules (`ControlLoop::step()` and everything it calls)

- No heap allocation, no logging, no exceptions. Fixed-size Eigen types only; no
  `std::vector` or dynamic matrix is created per cycle. Tests define
  `EIGEN_RUNTIME_NO_MALLOC` to catch violations.
- Never write NaN to the HAL. Commands are clamped to `tau_max` from the URDF.
- Every hot-path change reports `cobot_bench timing` numbers before and after (p50, p95)
  on the desktop.
- Keep `deterministic: true` working: one thread, fixed `dt`, fixed seeds, so two runs give
  byte-identical CSV (E2E-03).

## Tests

- Levels: unit (`tests/unit/<module>`), component with a known answer
  (`tests/component/`), end-to-end scenarios (`tests/e2e/scenarios/*.yaml`, run by
  `tools/run_e2e.py`), Python (`tests/python/`). Thresholds live in the scenario files,
  not in code.
- Every analytic derivative has a test against central differences (relative error
  < 1e-6). Dynamics is checked without Pinocchio by the six checks of ADR-004 (Jacobian,
  M symmetric positive definite, RNEA = M q̈ + C q̇ + G, Ṁ − 2C skew-symmetric, energy
  conservation, KDL fixture).
- Random tests use fixed seeds.
- `tests/unit/` and `tests/component/` pick up every subdirectory that has a
  `CMakeLists.txt`; add a directory, never edit a shared CMake list.

### Test-first, frozen tests

1. A feature starts with a test commit `test(<scope>): ...` written from the design and
   the interface headers. The tests must compile and must fail because the implementation
   is missing.
2. The implementation commits (`feat(<scope>): ...`, `fix(<scope>): ...`) never modify,
   delete or rename anything under `tests/`.
3. CI enforces this (`tools/ci/check_frozen_tests.sh`, job `frozen-tests`):
   - files under `tests/` that exist on `main` may not be changed or deleted in a PR
     unless the PR carries the `test-change` label and states the reason;
   - inside a PR, no commit after the last `test(` commit may touch `tests/`.
4. If a test looks wrong against the design, stop and report it. A separate PR labelled
   `test-change` fixes it. Never relax a test or a threshold to make code pass; G3 and
   spec 11.3 thresholds change only with the owner's agreement.

## Python (src/eval/, tools/, src/sim/launch)

- Python 3.12, type hints, `ruff` for lint and format (line length 100), `pytest`. One
  `pyproject.toml` at the root.
- Scripts take paths as arguments and write results to `results/<run-id>/` or to a path
  given as argument. They never write into the source tree.

## Build and run (dev machine: Windows + WSL Ubuntu-24.04)

The C++ toolchain lives in WSL. From PowerShell run the scripts as
`wsl -d Ubuntu-24.04 -- bash <repo>/tools/wsl/<script>.sh ...`.

| Command | What it does |
| --- | --- |
| `tools/wsl/setup.sh` | prints the `sudo apt install` line to type, creates the venv (`$COBOT_ROOT/venv`), installs the Python package |
| `tools/wsl/build.sh [--asan] [--target T]` | configure + build + ctest with `-Werror`; `--asan` builds Debug with ASan/UBSan |
| `tools/wsl/pytest.sh` | runs `pytest` in `.venv` |
| `tools/wsl/lint.sh` | clang-format, clang-tidy on changed files, ruff check and format |
| `tools/wsl/sim.sh {build\|check\|smoke}` | colcon build of `src/sim/`, offline asset check, headless Gazebo smoke test |
| `tools/wsl/prune.sh [--dry-run]` | removes `$COBOT_ROOT/build` and `colcon` entries of deleted branches; run after a merge |
| `source tools/wsl/env.sh` | ROS 2 + colcon workspace, with this project's own ROS domain (42) and Gazebo partition |

Do not pass inline `bash -lc "..."` commands containing `(`, `)` or `$` through `wsl`; put
them in a script file.

## Git and PRs

- Trunk-based. `main` is protected: PR required, required checks green on the PR head, rules
  apply to admins, no direct or force push. Branches need not be up to date with `main`; CI
  re-runs on `main` after every merge, and a red `main` is fixed before any other merge.
- One feature per PR, on a branch `feat/<feature>` (or `fix/`, `docs/`, `chore/`).
- Size limit: at most 400 changed lines of non-test code and at most 400 lines of tests.
  Pure moves, deletions, binary meshes and docs prose (ADRs, derivations, README) do not
  count.
- Conventional Commits: `feat(dynamics): ...`, `fix(planning): ...`, `test(control): ...`,
  `perf:`, `docs:`, `build:`, `ci:`, `chore:`.
- Push only when `build.sh`, `pytest.sh` and `lint.sh` pass locally.
- The PR description states what changed, which tests cover it, and the effect on the G3
  numbers and step time, with numbers or "no change expected". Gazebo demo runs attach
  the CSV report and plots.
- Required checks: `lint`, `frozen-tests`, `core`, `sanitizers`, `coverage` (≥ 80 % lines
  of `src/core/`), `python`, `e2e`, `gazebo`, `arm64`. A job with nothing to run yet still
  exists and passes, so the list never changes. `nightly` (E2E-05/06) is scheduled and not
  required; `deploy` pushes the Pi 5 image to GHCR on every push to `main`.
- Squash merge only, and only when every required check is green. The branch is deleted
  after merge.
- Every review finding is fixed before merge; nothing is parked for later.
- History carries only the owner: commits are authored by the owner, with no automated
  co-author lines and no tool signature lines in commit messages, PR or issue bodies.
  Local tool and editor files stay out of the repo through `.git/info/exclude`, not
  `.gitignore`.
- No code, data or hardware from an employer is used in this repo.

## Definition of done for a feature

1. The derivation is in `docs/derivations/`, and an ADR exists if a design choice was made.
2. Unit tests and a known-answer component test pass; the frozen-test rules held.
3. New behaviour is behind a YAML key that defaults to the previous behaviour until it is
   evaluated.
4. All required CI checks are green; E2E numbers are in the PR.
5. Hot-path changes attach `cobot_bench timing` numbers.
6. README or docs are updated if behaviour or parameters changed.
