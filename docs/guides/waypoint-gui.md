# Waypoint GUI (RViz2 panel)

An RViz2 panel (`cobot_gui/WaypointPanel`, package `src/ros2_gui/`) edits a waypoint list and drives
the external command mode ([build.md](build.md#external-command-mode), ADR-018). After
`sim.sh build` and `source tools/wsl/env.sh`:

```bash
ros2 launch cobot_sim sim.launch.py controller:=cobot_controller rviz:=true [gui:=true]
```

`rviz:=true` opens RViz2 with `src/sim/rviz/waypoints.rviz` (robot, TF, interactive markers, the
preview and reference paths, the panel docked) and starts the controller with
`command_source:=external` unless `command_source` is given. Workflow:

1. Add: drag the 6-DOF marker and press **Add from marker**, or edit the table cells
   (x, y, z in m; roll, pitch, yaw in degrees, `R = Rz(yaw) Ry(pitch) Rx(roll)`; duration in s,
   0 = from the Cartesian limits; stop). Selecting a row puts the 6-DOF marker on it and dragging
   moves that point; **Up**, **Down** and **Delete** reorder or remove it. **Load** and **Save**
   read and write the waypoint file (`docs/conventions.md` "Waypoint file").
2. Preview: validates the list from the current standby pose without moving; the path is drawn
   (`/rviz/preview_path`) and points with issues turn red with the reason in a tooltip and in
   the status area.
3. Run: enabled only after an ok preview of the unchanged list; any edit disables it again.
   The status area shows the mode, segment, time and tip errors, then the result.
4. Stop: cancels the goal; the arm decelerates along the path and waits in standby.
5. Reset: leaves the safety hold once its cause is gone (`/cobot_controller/reset`).

## Studio bridge, headless

The studio bridge (ADR-022, `docs/conventions.md` "Studio bridge") is served by the controller
with `studio:=true` (`command_source:=external log_mode:=recorder_live` unless given) and driven
headless with `cobot_bridge_cli`; while it holds authority the panel's goals are refused:

```bash
ros2 launch cobot_sim sim.launch.py controller:=cobot_controller studio:=true
ros2 run cobot_ros2 cobot_bridge_cli cobot_controller claim jog-joint:1:+:0.5 \
  move-joint:0,30,0,-90,0,60,0 waypoints:tests/e2e/waypoints/e2e_13.yaml export:/tmp/seg.csv release
```

Without Gazebo, `cobot_bench serve --config config/default.yaml --imperfections
config/imperfections/ideal.yaml [--name cobot_bench] [--out DIR] [--duration-s S]` runs the same
loop paced at 1 kHz against SimPlant and serves `/cobot_<name>` until Ctrl-C (Run CSV in
`DIR/run.csv`, default `results/serve-<YYYYmmdd-HHMMSS>`); `cobot_bridge_cli <name> ...` drives it.
