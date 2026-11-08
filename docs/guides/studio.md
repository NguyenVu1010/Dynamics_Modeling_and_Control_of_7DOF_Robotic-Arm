# cobot_studio

A Qt5 desktop app over `cobot_studio_core` ([ADR-021](../adr/021-cobot-studio-app.md)): a
Configuration tab (base pair, parameter overrides, controller gains, tolerances, studio file,
export for `cobot_bench`), a Tracking tab (bench runs and sweeps on a worker thread, overlaid
charts, an error table that recomputes on tolerance edits, Run CSV and Summary CSV export) and a
Verification tab (`cobot_bench verify` in process). It is off by default:

```bash
sudo apt install qtbase5-dev libqt5charts5-dev qt3d5-dev
cmake -S . -B build -DCOBOT_BUILD_STUDIO=ON && cmake --build build   # or: tools/wsl/build.sh --studio
build/src/studio/app/cobot_studio [--config-dir config] [--work-dir ~/.cache/cobot_studio/runs]
```

Under WSL the window opens through WSLg (Windows 11, no X server needed). Each run materialises
its inputs under the work dir, never in the repository. The source stays MIT; the `cobot_studio`
binary links Qt Charts (GPLv3), so the binary as distributed is under GPLv3.

## 3D view

The Tracking tab also has a 3D view (Qt3D) that replays the checked runs at the chart's time
cursor: the selected run solid, up to three other checked runs as ghosts in their colours, the
plant's true joint angles when the run has them (else the logged `q`). The selected run shows
its tip trace, the dashed desired path, the goal points, a sphere of the goal tolerance and a
red cross at the hold time. Play (0.25x, 1x, 4x) and the time slider move the same cursor as the
chart; the magnify box (x1 to x1000) scales the tip's deviation from the desired path and the
sphere together. The meshes of `src/sim/meshes` are read once, off the GUI thread. The 3D test
runs under `xvfb-run` when it is installed (`xvfb libgl1-mesa-dri`) and skips without OpenGL.

## Control tab

The Control tab (between Configuration and Tracking) drives a controller through the studio
bridge `/cobot_<name>`, chosen with `--bridge <name>` (default `cobot_controller`, the Gazebo
controller's node name). "Take control" claims authority. The joint page jogs a joint while its
`-` / `+` button is held and moves to typed-in goals (bounded by the jog band of the session's
limits); the TCP page jogs and moves the TCP in the base or tool frame; the waypoint page teaches
the current TCP pose, opens and saves waypoint files, previews them (issues per waypoint, the
planned path in the live 3D view) and runs them. Stop, E-stop and Reset stay visible below the
pages, next to a speed slider shared with the Configuration tab. The heartbeat runs on the GUI
timer on purpose: a frozen window stops the robot and gives authority back; closing the window
releases it. Every finished motion arrives in the Tracking tab as a live run ("live move_joint
#1") read from the Recorder segment the host exports (the motion plus a 1 s settle window), with
the same charts, error table and 3D replay as bench runs. "Apply to Gazebo" in the Configuration
tab materialises the configuration and applies it while the loop is in standby; the rows the
Gazebo path ignores (payload, friction, transmission, position source, timing, comm) are greyed
while connected.

```bash
# Against the bench host (no Gazebo), in two terminals:
build/src/bench/cobot_bench serve --config config/default.yaml \
  --imperfections config/imperfections/ideal.yaml --name cobot_bench
build/src/studio/app/cobot_studio --bridge cobot_bench
# Against Gazebo:
ros2 launch cobot_sim sim.launch.py controller:=cobot_controller studio:=true
build/src/studio/app/cobot_studio   # --bridge cobot_controller
```

## Manual control

1. Start a host. For Gazebo, run `ros2 launch cobot_sim sim.launch.py controller:=cobot_controller
   studio:=true [gui:=true] [imperfections_path:=<profile>] [seed:=N]` (after `sim.sh build` and
   `source tools/wsl/env.sh`). Without Gazebo, run `cobot_bench serve --config
   config/default.yaml --imperfections <profile> [--seed N]` (region `/cobot_cobot_bench`).
2. Start `cobot_studio --bridge cobot_controller` (or `--bridge cobot_bench`). The status bar
   shows "Bridge: connected".
3. In the Control tab, press **Take control**. The jog and Go buttons are enabled only while the
   studio holds authority. **Release** gives authority back, and so does closing the window.
4. The three pages:
   - **Joint**: hold `-` / `+` to jog one joint. **Copy current**, edit the goals and press
     **Go** for a joint move.
   - **TCP**: the same in the base or the tool frame. A goal pose is a `move_tcp`.
   - **Waypoints**: **Add current pose**, **Open...** / **Save...** a waypoint file,
     **Preview** (issues per waypoint, the path in the 3D view), then **Run**.
5. Below the pages:
   - **Stop** decelerates to standby.
   - **E-stop** holds at once (`estop`). **Reset** leaves the hold.
   - The speed slider (default 10 %) scales the joint and TCP limits of jogs and moves.
     Waypoint runs keep their validated durations.
6. Each finished motion appears in the Tracking tab as a live run, for example "live move_joint
   #2", with the same charts, error table and 3D replay as a bench run.
7. **Apply to Gazebo** in the Configuration tab applies the edited configuration while the loop
   is in standby. While connected, the rows the Gazebo path ignores are greyed: payload,
   friction, transmission, position source, timing and comm.

![Joint page after a joint move in Gazebo](../images/studio-control-joint.png)
![Waypoint page with a previewed list](../images/studio-control-waypoints.png)
![Tracking tab with four live runs from Gazebo](../images/studio-tracking-live.png)

To drive a host headless (CI, scripts), use one line per session, for example:
`cobot_bridge_cli cobot_bench claim move-joint:0,30,0,-90,0,60,0 release`. On Gazebo, use `ros2
run cobot_ros2 cobot_bridge_cli cobot_controller ...`.

## Verification tab

The Verification tab (the fourth tab) runs `cobot_bench verify` in process
([ADR-023](../adr/023-hardware-scenario-verification.md)): the same loading, checks and messages,
the same engine (`bench::compare`) and the same writer (`bench::write_verify_outputs`), so a job
writes the bytes the CLI writes for the same arguments. Inputs, on the left:

- **Hardware models** (`hardwareList`): one checkable row per `config/hardware/*.yaml`.
- **Algorithms** (`algorithmList`): the registered algorithms (`computed_torque`, `pd_gravity`,
  `drive_position`).
- **Task** (`taskCombo`): the `config/tasks/*.yaml` files that load as a task (a waypoint file
  does not).
- **Runs** (default 100), **Jobs** (0 = auto), **Quick**, **Tune the gain first** and
  **Sensitivity of the assumptions** (both on): the CLI's `--runs`, `--jobs`, `--quick`,
  `--no-tune` and `--no-sensitivity`.
- **Pairs**: the cross product of the checked models and algorithms. A pair the model does not
  list is greyed: it is a "not listed" comparison row, not an error.

**Run** starts the job on the studio's worker thread. The phase line shows "tune", "Monte
Carlo", "sensitivity" or "ablation" with the pair (i of n) and the progress bar the runs of that
phase. **Cancel** stops before the next run; the tab goes back to idle with "cancelled" and no
file is written. A job writes into `<work-dir>/verify_NNNN/` (the first free number), never into
the repository; an input error (unknown file, no listed pair) shows the CLI's message and writes
nothing.

The result area shows the selected pair (**pairCombo** when several pairs ran). Nothing in it is
editable: labels and read-only tables only.

- The verdict badge: PASS (green), FAIL (red) or MARGINAL (amber), with "(quick)" after a quick
  job, and the disclaimer under it.
- The pass count over the runs, the pass rate, the Wilson 95 % interval and the hold count.
- Worst values: metric, worst value over the failed runs, threshold, margin limit and a bar of
  the value against the threshold.
- Assumptions: key, kind, range or scale, sensitivity and the pass counts at base, low and high.
- The gain line (key, value used, source) and the tune table, the chosen level highlighted.
- Causes (ablation) and suggestions, on a FAIL only.
- The comparison table, when more than one pair: one row per pair, "not listed" included.

**Open report** opens the pair's `report.md`; **Export...** copies the job's directory into an
empty or new folder. The tab reads its numbers from the same `ComparisonRow` structs the reports
are written from.

## Safety

From [ADR-022](../adr/022-shm-bridge-and-manual-control.md):

- Heartbeat: the studio beats every 50 ms. If no beat arrives for 200 ms (a frozen or killed
  studio), the motion stops with deceleration and authority goes back.
- Deadman: a jog not refreshed for 100 ms decelerates to zero.
- Jog band: jogs and joint moves stop at the soft margin + 2° (5° from the URDF limits by
  default), never in the holding band.
- One command source: while the studio holds authority, the RViz panel's goals and `~/reset`
  are refused ("studio is in control").

Measurements: [`docs/results/studio/`](../results/studio/README.md) (PF-02, stop
guarantee, sample live runs).
