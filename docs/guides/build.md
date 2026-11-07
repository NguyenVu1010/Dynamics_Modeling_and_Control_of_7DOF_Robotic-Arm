# Build and run

All C++ builds run in WSL Ubuntu-24.04 (ROS 2 Jazzy and Gazebo Harmonic installed). From
PowerShell: `wsl -d Ubuntu-24.04 -- bash <repo>/tools/wsl/<script>.sh`.

Everything the scripts generate lives in one folder per project, `~/projects/7dof_dynamic_control/`
(override with `COBOT_ROOT`): `build/<branch>[-asan|-cov]`, `colcon/<branch>`, `venv`, `out`.
`tools/wsl/prune.sh` removes the builds of branches that no longer exist.

```bash
tools/wsl/setup.sh                 # prints the apt line to type, creates the venv
tools/wsl/build.sh                 # CMake + Ninja, -Werror, ctest
tools/wsl/build.sh --asan          # Debug with ASan + UBSan
tools/wsl/build.sh --studio        # also builds cobot_studio (Qt5)
tools/wsl/build.sh --target T      # one target
tools/wsl/pytest.sh                # Python tests
tools/wsl/lint.sh                  # clang-format, clang-tidy, ruff
tools/wsl/sim.sh build             # colcon build of src/sim, src/ros2_interfaces, src/ros2_node, src/ros2_gui
tools/wsl/sim.sh check             # xacro + check_urdf + gz sdf, no Gazebo needed
tools/wsl/sim.sh test              # colcon test of cobot_gui (panel data model)
tools/wsl/sim.sh smoke             # headless Gazebo, 20 s
source tools/wsl/env.sh            # ROS 2 + workspace, own ROS domain (42) and Gazebo partition
```

`env.sh` keeps this project apart from other ROS 2 projects in the same WSL, which use the
default domain 0: override with `COBOT_ROS_DOMAIN_ID` / `COBOT_GZ_PARTITION`. `sim.sh build`
wipes the colcon workspace when it was built from another source directory (a deleted worktree).

## The controller in Gazebo

After `sim.sh build` and `source tools/wsl/env.sh`:

```bash
ros2 launch cobot_sim sim.launch.py controller:=cobot_controller [gui:=true]   [config_path:=config/default.yaml] [imperfections_path:=config/imperfections/realistic.yaml]   [seed:=7] [log_path:=results/<run-id>/run.csv]
python tools/run_e2e.py --all --tag gazebo --out results/e2e-gazebo   # E2E-07, 09, 11, 11b
```

The controller's log lives in the shared-memory region `/cobot_log_cobot_controller` and survives a
crash of the controller or of Gazebo. `ros2 service call /cobot_controller/dump_log
std_srvs/srv/Trigger` appends the rows not yet written to `log_path` (`log_stream_ms:=200` does it
every 200 ms); after a crash, `ros2 run cobot_ros2 cobot_logdump cobot_controller [--out file.csv]`
exports what the region still holds, and the next activation writes any leftover rows to
`<log_path>.recovered-<YYYYmmdd-HHMMSS>.csv` before it starts.

## External command mode

`command_source:=external` (default: the config file's `cobot.command_source`) starts the arm in
standby at the measured pose and offers, under `/cobot_controller/`, the action
`follow_waypoints`, the services `preview_waypoints` (validation only, no motion) and `reset`
(std_srvs/Trigger, leaves the safety hold once its cause is gone), and the latched
`reference_path` of the running goal. A goal is accepted only in standby; it is validated from
the standby pose, and canceling it stops along the path. Client:

```bash
ros2 run cobot_ros2 send_waypoints.py tests/e2e/waypoints/e2e_10.yaml [--preview] [--cancel-after 3]
```

The RViz2 panel that drives this mode: [waypoint-gui.md](waypoint-gui.md). The Qt desktop app:
[studio.md](studio.md).
