# cobot_studio phase B: results

Run on `main` at 499642b (after #64–#68) on 2026-11-06. Machine: Intel Core i5-14400F (12
logical CPUs visible to WSL), WSL2 Ubuntu-24.04, kernel `6.18.54.1-microsoft-standard-WSL2`,
Release build (`tools/wsl/build.sh --studio`). **WSL2, normal priority, no RT**: no
`SCHED_FIFO`, no isolated cores, the desktop idle otherwise. Design:
`docs/design/2026-11-02-cobot-studio-design.md` 7; decision: ADR-022.

## PF-02: control step with the studio bridge

`cobot_bench timing --paced` runs the loop at 1 kHz with `clock_nanosleep`; with
`recorder_live` a reader drains the state ring on another CPU. Median of 5 runs of 10 000 steps
for each mode (µs; `late_p99` is the p99 of the wake-up lateness, only reported).

| Host | `--log-mode` | p50 | p99 | max | late_p99 | late cycles | reader_lost | reader_cpu |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| desktop, WSL2 | off | 40.7 | 108.4 | 594 | 426 | 9 | – | shared |
| desktop, WSL2 | recorder | 39.2 | 110.2 | 654 | 478 | 2 | – | shared |
| desktop, WSL2 | recorder_live | 5.4 | 21.2 | 137 | 183 | 3 | 0 of 10 000 | other |
| Pi 5 | all | pending hardware | | | | | | |

Gate (CI, job `core`): p99 < 1 ms and no allocation. Every run passes it on the desktop.

The paced `off` and `recorder` rows are slower than `recorder_live`. This comes from WSL2,
not from the loop. With nothing else running, the vCPU idles for about 1 ms between steps, and
each step starts on a cold core. The `recorder_live` reader keeps another CPU busy, which
avoids that. The same `recorder` run with a busy loop pinned to CPU 11 (`taskset -c 11 sh -c
'while :; do :; done'`) gives p50 5.5–5.9 µs and p99 23–31 µs over 3 runs, the same level as
`recorder_live`. So the paced rows compare WSL2 idle states and say nothing about the bridge.

The cost of the bridge is measured unpaced, where every mode runs on a busy core (same command
without `--paced`, median of 5):

| `--log-mode` | p50 | p95 | p99 | max |
| --- | --- | --- | --- | --- |
| off | 4.75 | 5.01 | 5.85 | 47 |
| recorder | 4.74 | 5.00 | 6.58 | 74 |
| recorder_live | 4.91 | 5.28 | 6.15 | 96 |

`recorder_live` costs 0.17 µs at p50 and 0.28 µs at p95 more than `recorder` (one record copy
and the ring counters). That is inside ADR-022's estimate of 0.05–1 µs per step, about 0.02 %
of the 1000 µs period.

PF-01 for reference, same machine, unpaced, as the CI gate measures it (`--steps 10000`, median
of 5): p50 4.73 µs, p95 4.98 µs, max 69 µs (gate p95 < 1 ms; desktop target < 200 µs).

## Stop guarantee

Setup: `cobot_bench serve` (default + ideal), and a CLI running `claim
move-joint:60,30,0,-90,0,60,0 release` at 10 % speed. The CLI is frozen with `kill -STOP` 1.8 s
after it starts, during the 2.5 s move. Two times are taken, both on `CLOCK_MONOTONIC`:

- The last heartbeat: the owner's beat stamp `owner_beat_ns`, read with `od` from the region
  file `/dev/shm/cobot_cobot_bench` (offset 72 of the header) right after the SIGSTOP. A frozen
  CLI beats no more, so this is its last beat.
- Standby: the moment a second, read-only `cobot_bridge_cli cobot_bench wait-standby` returns,
  stamped by a Python wrapper (`time.monotonic_ns()`). Its poll period is 5 ms.

The commands are in "Reproduce" below. 5 trials:

| Trial | last beat → standby |
| --- | --- |
| 1 | 0.706 s |
| 2 | 0.706 s |
| 3 | 0.709 s |
| 4 | 0.705 s |
| 5 | 0.707 s |

The maximum is 0.709 s. The bound is 0.2 s (heartbeat timeout) + 0.005 s (bridge poll) +
0.5 s (`safety.stop_time_s`) = 0.705 s, plus up to 5 ms for the observer's own poll and the
wrapper's exit. So the measurement stays within the bound at the observer's resolution. The Run
CSV of an earlier series of 5 trials shows the deceleration ramp lasting 0.499 s each time. Each
frozen CLI, once resumed, printed `move-joint:...: stopped (stopped)` and exited 1. The next
`claim` from a new CLI succeeded, which shows the authority was handed back.

## Sample live runs

These are the E2E-13 motion steps, run through the studio GUI on 2026-11-06:

1. Hold J1 `+` for 0.5 s.
2. Move the joints to home (0, 30, 0, -90, 0, 60, 0)°.
3. TCP move to the home TCP + 50 mm in x.
4. The two waypoints of `tests/e2e/waypoints/e2e_13.yaml` (preview, then run).

Settings: `default.yaml` + `realistic.yaml`, seed 7, speed 10 %, default tolerances (goal 1 mm
/ 0.5°). The values below are read from the Tracking table.

| Host | Live run | Final pos (mm) | Final rot (°) | Path max pos (mm) | Path max rot (°) | Settle (s) | Held |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `cobot_bench serve` | jog_joint #1 | 0.046 | 0.0088 | 0.130 | 0.015 | 0 | no |
| `cobot_bench serve` | move_joint #2 | 0.115 | 0.0103 | 0.200 | 0.013 | 0.001 | no |
| `cobot_bench serve` | move_tcp #3 | 0.097 | 0.0158 | 0.196 | 0.030 | 0 | no |
| `cobot_bench serve` | waypoints #4 | 0.074 | 0.0065 | 0.161 | 0.014 | 0 | no |
| Gazebo | jog_joint #1 | 0.054 | 0.0093 | 0.118 | 0.022 | 0.001 | no |
| Gazebo | move_joint #2 | 0.051 | 0.0130 | 0.146 | 0.029 | 0 | no |
| Gazebo | move_tcp #3 | 0.060 | 0.0102 | 0.149 | 0.025 | 0 | no |
| Gazebo | waypoints #4 | 0.033 | 0.0101 | 0.133 | 0.017 | 0 | no |

Every run ends within the goal and path tolerances, without a hold. The table shows the bench
move_tcp settle time as `7.105e-15`, a rounding residue of 0. The same steps driven headless
(`cobot_bridge_cli`, one `export:` after each move) give results of the same size on both hosts:

- final position 0.02–0.11 mm;
- path maximum 0.12–0.19 mm;
- settle 0 s;
- no hold;
- move durations 0.63 s (move_joint), 3.76 s (move_tcp), 1.55 s (waypoints).

The segment CSVs are 3–11 MB each (114 columns at 1 kHz over the motion and 1 s of settle), so
only this summary is kept here. Screenshots: [joint page](../../images/studio-control-joint.png),
[waypoint page](../../images/studio-control-waypoints.png), [Tracking tab with the Gazebo live
runs](../../images/studio-tracking-live.png).

## Notes

- The screenshots were taken from the studio running on an Xvfb display (1600×1000, no window
  manager) against headless Gazebo. The steps were driven with XTest events from a script. The
  WSLg window looks the same apart from the title bar.

## Known limitations

- In standby on the realistic profile, the TCP pose holds within 0.05 mm while the joints drift
  slowly in the null space (about 0.007°/s; the standby hold has no posture term). A
  `move_joint` brings the joints back. Tracked in
  [#69](https://github.com/NguyenVu1010/Dynamics_Modeling_and_Control_of_7DOF_Robotic-Arm/issues/69).

## Reproduce

All commands run from the repository root in WSL. `B` is the Release build of
`tools/wsl/build.sh` (`build/<branch>` under `~/projects/7dof_dynamic_control`, here `main`).

```bash
B=~/projects/7dof_dynamic_control/build/main/src/bench
OUT=~/projects/7dof_dynamic_control/out/studio
mkdir -p $OUT
# PF-02, PF-01 (repeat 5 times, take the median)
for m in off recorder recorder_live; do
  $B/cobot_bench timing --config config/default.yaml --steps 10000 --paced --log-mode $m
done
$B/cobot_bench timing --config config/default.yaml --steps 10000
# Stop guarantee (one trial): host in the background, a CLI frozen with kill -STOP mid-move
$B/cobot_bench serve --config config/default.yaml \
  --imperfections config/imperfections/ideal.yaml --out $OUT/stop > /dev/null & srv=$!
sleep 2
$B/cobot_bridge_cli cobot_bench claim move-joint:60,30,0,-90,0,60,0 release & cli=$!
sleep 1.8; kill -STOP $cli
beat=$(od -An -t d8 -j 72 -N 8 /dev/shm/cobot_cobot_bench)  # owner_beat_ns
standby=$(python3 -c 'import subprocess, sys, time
subprocess.run(sys.argv[1:], check=True, stdout=sys.stderr); print(time.monotonic_ns())' \
  $B/cobot_bridge_cli cobot_bench wait-standby)
echo "last beat -> standby: $(( (standby - beat) / 1000000 )) ms"
kill -CONT $cli; wait $cli    # prints "stopped (stopped)", exit 1
$B/cobot_bridge_cli cobot_bench claim release
kill -INT $srv; wait $srv
# Live runs, headless (realistic, seed 7)
$B/cobot_bench serve --config config/default.yaml \
  --imperfections config/imperfections/realistic.yaml --seed 7 --out $OUT/live > /dev/null & srv=$!
sleep 2
$B/cobot_bridge_cli cobot_bench claim jog-joint:1:+:0.5 move-joint:0,30,0,-90,0,60,0 \
  export:$OUT/mj.csv move-tcp:0.160012653,-0.806959296,0.389122496,0,-90,90 \
  export:$OUT/mt.csv waypoints:tests/e2e/waypoints/e2e_13.yaml:2 export:$OUT/wp.csv release
kill -INT $srv; wait $srv
```

On Gazebo: `ros2 launch cobot_sim sim.launch.py controller:=cobot_controller studio:=true
imperfections_path:=$PWD/config/imperfections/realistic.yaml seed:=7`, then the same CLI steps
through `ros2 run cobot_ros2 cobot_bridge_cli cobot_controller ...`, or `cobot_studio`
([studio guide](../../guides/studio.md)).
