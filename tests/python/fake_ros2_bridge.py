"""Stand-in for the `ros2` CLI with the studio bridge (feature controller-bridge); stdlib only.

Used by test_run_e2e_gazebo_bridge.py. Every call is logged as a JSON line {"argv", "pid"} to
FAKE_ROS2_LOG. Flags come from FAKE_ROS2_MODE (comma separated).

`ros2 launch ...`: writes the header and the first half of a Run CSV (current layout, `cmd`
column) at `log_path:=` at once, the rest after SIGINT or SIGTERM, then logs {"signal"} and exits
0 (exit 4 after FAKE_ROS2_MAX_S seconds without a signal). 200 rows, t_ns = k * 10 ms (0 .. 1.99 s),
q = qref, qd = 0, tau = 1 N*m, tip on its reference. Modes and `cmd` in order: standby/waypoints,
tracking/jog_joint, standby/jog_joint, tracking/move_joint, standby/move_joint, tracking/move_tcp,
standby/move_tcp, tracking/waypoints, standby/waypoints. Flags: crash = exit 3 at once;
nomovetcp = the move_tcp rows log move_joint; holding = row 150 in holding (hold estop);
tracklast = the last row in tracking. While a launch runs it keeps FAKE_ROS2_LOG + ".launch".

`ros2 topic echo ... /cobot_controller/status`: no launch -> exit 1; otherwise `idle` for the first
FAKE_ROS2_STANDBY_AFTER (default 2) messages, then `standby` (bare value with `--field data`).
Logged as {"status": [modes printed]}.

`ros2 run cobot_ros2 cobot_bridge_cli <name> <steps...>`: logged as {"bridge": argv after `run`,
"launch_running": bool}; keeps FAKE_ROS2_LOG + ".bridge" while it runs. Prints one line per step,
`<k> <step>: ok` (flushed); `pause:<s>` sleeps s seconds; `export:<csv>` writes a segment CSV there
(the waypoints rows: tracking then standby, `cmd` waypoints, tip on its reference; flag segfail =
the tip 10 mm off its reference on every row). FAKE_BRIDGE_EXIT (default 0) != 0: the last step
prints `<k> <step>: failed: held: estop` and the CLI exits with that code.

`ros2 run cobot_ros2 send_waypoints.py FILE`: the RViz panel client; logged as {"panel": argv after
`run`, "bridge_running": bool, "launch_running": bool}; prints `goal rejected: studio is in control`
(flag panelquiet: prints `goal rejected`) and exits FAKE_PANEL_EXIT (default 1; 0 prints
`status: succeeded`).

`ros2 service call /cobot_controller/dump_log ...`: logged as {"dump_log": argv after `call`},
exit 0.
`ros2 run cobot_ros2 cobot_logdump <controller> --out CSV`: logged as {"logdump": argv after `run`},
writes the whole Run CSV of the last launch to CSV, exit 0.
`ros2 control list_controllers` reports cobot_controller active; anything else exits 0.
"""

import json
import os
import signal
import sys
import time
from pathlib import Path

DT_NS = 10_000_000
PD = [0.13, -0.78, 0.43]
N = 7
CHANNELS = ("qddref", "qraw", "qt", "qdt", "qddt", "qmt", "taum", "tauf")
STATUS_TOPIC = "/cobot_controller/status"
PLAN = [
    (10, "standby", "waypoints"),
    (20, "tracking", "jog_joint"),
    (10, "standby", "jog_joint"),
    (20, "tracking", "move_joint"),
    (10, "standby", "move_joint"),
    (20, "tracking", "move_tcp"),
    (10, "standby", "move_tcp"),
    (40, "tracking", "waypoints"),
    (60, "standby", "waypoints"),
]


def header() -> str:
    cols = ["t_ns", "mode"]
    cols += [f"{p}{j}" for p in ("q", "qd", "qref", "qdref", "tau") for j in range(1, N + 1)]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    cols += [f"e{k}" for k in range(1, 7)] + ["step_us", "hold"]
    cols += [f"{p}{j}" for p in CHANNELS for j in range(1, N + 1)]
    return ",".join(cols + ["cmd"])


def row(k: int, mode: str, cmd: str, hold: str = "none", tip_err: float = 0.0) -> str:
    q = [0.1 * j for j in range(N)]
    p = [PD[0] + tip_err, PD[1], PD[2]]
    vals = [k * DT_NS, mode, *q, *[0.0] * N, *q, *[0.0] * N, *[1.0] * N, *p, *[0.0] * 3]
    vals += [*PD, *[0.0] * 3, *[0.0] * 6, 0, hold, *[0.0] * (len(CHANNELS) * N), cmd]
    return ",".join(str(v) for v in vals)


def run_rows(flags: set[str]) -> list[str]:
    rows = []
    for count, mode, cmd in PLAN:
        logged = "move_joint" if cmd == "move_tcp" and "nomovetcp" in flags else cmd
        rows += [(mode, logged, "none")] * count
    if "holding" in flags:
        rows[150] = ("holding", rows[150][1], "estop")
    if "tracklast" in flags:
        rows[-1] = ("tracking", rows[-1][1], "none")
    return [row(k, *r) for k, r in enumerate(rows)]


def segment_rows(flags: set[str]) -> list[str]:
    err = 0.01 if "segfail" in flags else 0.0
    out = [row(k, "tracking", "waypoints", tip_err=err) for k in range(110, 150)]
    out += [row(k, "standby", "waypoints", tip_err=err) for k in range(150, 200)]
    return out


def append(log: Path, entry: dict) -> None:
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry) + "\n")


def launch(argv: list[str], flags: set[str], log: Path) -> int:
    stop: list[int] = []

    def on_signal(signum: int, _frame: object) -> None:
        stop.append(signum)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if "crash" in flags:
        return 3
    args = dict(a.split(":=", 1) for a in argv if ":=" in a)
    marker = Path(str(log) + ".launch")
    Path(str(log) + ".echo").unlink(missing_ok=True)
    marker.write_text("running", encoding="utf-8")
    rows = run_rows(flags)
    half = len(rows) // 2
    csv = None
    if "log_path" in args:
        path = Path(args["log_path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        csv = path.open("w", encoding="utf-8")
        csv.write(header() + "\n")
        csv.writelines(r + "\n" for r in rows[:half])
        csv.flush()
    deadline = time.monotonic() + float(os.environ.get("FAKE_ROS2_MAX_S", "120"))
    while not stop and time.monotonic() < deadline:
        time.sleep(0.05)
    marker.unlink(missing_ok=True)
    if not stop:
        return 4
    if csv is not None:
        csv.writelines(r + "\n" for r in rows[half:])
        csv.close()
    append(log, {"signal": stop[0]})
    return 0


def echo(argv: list[str], log: Path) -> int:
    if not Path(str(log) + ".launch").exists():
        time.sleep(0.2)
        append(log, {"status": []})
        return 1
    if STATUS_TOPIC not in argv:
        return 0
    counter = Path(str(log) + ".echo")
    count = int(counter.read_text(encoding="utf-8")) if counter.exists() else 0
    counter.write_text(str(count + 1), encoding="utf-8")
    mode = "idle" if count < int(os.environ.get("FAKE_ROS2_STANDBY_AFTER", "2")) else "standby"
    print(mode if "--field" in argv else f"data: {mode}")
    print("---", flush=True)
    append(log, {"status": [mode]})
    return 0


def bridge(argv: list[str], flags: set[str], log: Path) -> int:
    append(log, {"bridge": argv, "launch_running": Path(str(log) + ".launch").exists()})
    marker = Path(str(log) + ".bridge")
    marker.write_text("running", encoding="utf-8")
    code = int(os.environ.get("FAKE_BRIDGE_EXIT", "0"))
    steps = [a for a in argv[3:] if not a.startswith("--")]
    try:
        for k, step in enumerate(steps, 1):
            if code != 0 and k == len(steps):
                print(f"{k} {step}: failed: held: estop", flush=True)
                return code
            if step.startswith("pause:"):
                time.sleep(float(step.split(":", 1)[1]))
            elif step.startswith("export:"):
                out = Path(step.split(":", 1)[1])
                out.parent.mkdir(parents=True, exist_ok=True)
                rows = segment_rows(flags)
                out.write_text("\n".join([header(), *rows]) + "\n", encoding="utf-8")
                print(f"{k} {step}: ok {len(rows)} rows -> {out}", flush=True)
                continue
            print(f"{k} {step}: ok", flush=True)
        return code
    finally:
        marker.unlink(missing_ok=True)


def panel(argv: list[str], flags: set[str], log: Path) -> int:
    running = Path(str(log) + ".bridge").exists()
    launch_running = Path(str(log) + ".launch").exists()
    append(log, {"panel": argv, "bridge_running": running, "launch_running": launch_running})
    code = int(os.environ.get("FAKE_PANEL_EXIT", "1"))
    if code == 0:
        print("status: succeeded")
    elif "panelquiet" in flags:
        print("goal rejected")
    else:
        print("goal rejected: studio is in control")
    return code


def logdump(argv: list[str], flags: set[str], log: Path) -> int:
    append(log, {"logdump": argv})
    out = Path(argv[argv.index("--out") + 1])
    out.parent.mkdir(parents=True, exist_ok=True)
    rows = run_rows(flags)
    out.write_text("\n".join([header(), *rows]) + "\n", encoding="utf-8")
    print(f"{len(rows)} rows -> {out}")
    return 0


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_ROS2_MODE", "").split(",")))
    log = Path(os.environ["FAKE_ROS2_LOG"])
    argv = sys.argv[1:]
    append(log, {"argv": argv, "pid": os.getpid()})
    if argv[:1] == ["launch"]:
        return launch(argv, flags, log)
    if argv[:2] == ["topic", "echo"]:
        return echo(argv, log)
    if argv[:2] == ["service", "call"]:
        append(log, {"dump_log": argv[2:]})
        print("std_srvs.srv.Trigger_Response(success=True, message='run.csv 0 rows')")
        return 0
    if argv[:3] == ["run", "cobot_ros2", "cobot_logdump"]:
        return logdump(argv[1:], flags, log)
    if argv[:3] == ["run", "cobot_ros2", "cobot_bridge_cli"]:
        return bridge(argv[1:], flags, log)
    if argv[:3] == ["run", "cobot_ros2", "send_waypoints.py"]:
        return panel(argv[1:], flags, log)
    if argv[:2] == ["control", "list_controllers"]:
        print("joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active")
        print("cobot_controller        cobot_ros2/CobotController                    active")
    return 0


if __name__ == "__main__":
    sys.exit(main())
