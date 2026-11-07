# Environment configuration

Environment parameters are edited in config files, not code. The controller and the
environment are kept in separate files:

| File | Holds |
| --- | --- |
| `config/default.yaml` | the controller: URDF path, rate, trajectory, CLIK, computed-torque gains, friction compensation, safety |
| `config/<profile>.yaml` | the controller tuned for one hardware profile (`premium_fr3`, `mid_kr1205`, `budget_xarm7`; ADR-019) |
| `config/imperfections/*.yaml` | the environment (the robot as it really is): sensors, motors, friction, model mismatch, timing, seed |
| `src/sim/urdf/mass_budget.yaml` | masses and armature; `tools/fix_urdf_inertia.py` regenerates the URDF from it |

Profiles: `ideal.yaml` (perfect robot), `realistic.yaml` (typical industrial servo, the
default), `pessimistic.yaml`, and the three hardware-tier profiles of
[ADR-019](../adr/019-realism-2-hardware-profiles.md). Every group in a profile names the source of
its numbers.

- bench: `cobot_bench run --config config/default.yaml --imperfections config/imperfections/realistic.yaml [--seed N] --out run.csv`
- Gazebo: `ros2 launch cobot_sim sim.launch.py controller:=cobot_controller imperfections_path:=<profile> [seed:=N]`;
  without `imperfections_path` the realistic profile applies. Sensor and motor effects go
  through the controller's imperfect joint interface; friction goes into the physics (each
  joint's `<dynamics friction damping>`); model mismatch is emulated by perturbing the
  controller's model. `payload_kg` must be 0 on this path and timing jitter is ignored
  ([ADR-011](../adr/011-simulation.md)).
- Custom profile: copy `config/imperfections/realistic.yaml`, change the values (a scalar
  applies to every joint, a 7-element list sets each joint), update the `source` lines, and
  pass the new file to `--imperfections` or `imperfections_path:=`.

## Hardware models and tasks

Hardware verification asks "this hardware, this algorithm, this task" ([ADR-023](../adr/023-hardware-scenario-verification.md)).
Two more file types name those pieces. They only load and combine the files above; no command reads them yet:

| File | Holds |
| --- | --- |
| `config/hardware/<model>.yaml` | one piece of hardware: its profile and base controller file, torque levels, gains per algorithm, and the provenance (`published`, `derived` or `assumed`, with bounds) of every active imperfection key. A model with an unsourced key does not load. |
| `config/tasks/<task>.yaml` | a trajectory or a waypoint file, the payload, and the requirements of the task. |

The three models are `premium_fr3`, `mid_kr1205` and `budget_xarm7`; the tasks are `reference_move`
and `pick_place_3kg`. To add a model, copy one, point it at your profile and list every key the
loader says has no entry. The task thresholds and the provenance bounds are proposals awaiting the
owner's review. Schemas and rules: [Hardware model file](../conventions.md#hardware-model-file) and
[Task file](../conventions.md#task-file).

Key-by-key reference: [docs/conventions.md](../conventions.md#configuration-files).
