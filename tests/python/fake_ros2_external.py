"""Stand-in for the `ros2` CLI with the external command mode (feature waypoint-ros); stdlib only.

Superset of fake_ros2.py, used by test_run_e2e_gazebo_waypoints.py. Every call is logged as a
JSON line {"argv", "pid"} to FAKE_ROS2_LOG.

`ros2 launch ...`: as fake_ros2.py (header and first half of a Run CSV at `log_path:=` at once,
the rest after SIGINT or SIGTERM, then {"signal"} is logged and exit 0; exit 4 after
FAKE_ROS2_MAX_S seconds without a signal). Rows: t_ns = k * 1 ms up to FAKE_ROS2_T_END_S
(default 1.0 s), q = qref, qd = 0, tau = 1 N*m. First half: tip error 50 mm, mode tracking.
Second half: tip error from FAKE_ROS2_MODE (empty = 0, pos4mm = 4 mm, fail = 10 mm), mode
`standby` when the launch got `command_source:=external`, else `finished`.
Extra FAKE_ROS2_MODE flags (comma separated): nocsv = no CSV; crash = exit 3 at once;
moving = qd1 = 0.05 rad/s on the last 50 rows; overtau = tau1 = 1000 N*m on the first row;
holdend = second-half mode `holding` instead of `standby`.
While a launch runs it keeps a marker file (FAKE_ROS2_LOG + ".launch") holding its
command_source.

`ros2 topic echo ... /cobot_controller/status`: with no launch running, prints nothing and exits 1.
Otherwise the first FAKE_ROS2_STANDBY_AFTER (default 2) messages of this launch are `idle`, the
next ones `standby` (external) or `tracking` (config); FAKE_ROS2_MODE flag nostandby keeps
`idle`. With --once one message is printed, without it the messages up to and including the
first standby one (at most 50). Format as the real CLI: `data: <mode>` then `---`, or the bare
value with `--field data`. Logged as {"status": [modes printed]}.

`ros2 run cobot_ros2 send_waypoints.py FILE [...]`: logged as {"client": argv after `run`,
"launch_running": bool}; prints a result as YAML and exits FAKE_CLIENT_EXIT, default 3 when
--cancel-after is given (canceled as requested) else 0.

`ros2 control list_controllers` reports cobot_controller active; anything else exits 0.
"""

import json
import os
import signal
import sys
import time
from pathlib import Path

DT_NS = 1_000_000
PD = [0.4, 0.1, 0.5]
TRANSIENT_M = 0.05
STATUS_TOPIC = "/cobot_controller/status"


def header() -> str:
    cols = ["t_ns", "mode"]
    cols += [f"{p}{j}" for p in ("q", "qd", "qref", "qdref", "tau") for j in range(1, 8)]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    cols += [f"e{k}" for k in range(1, 7)] + ["step_us"]
    return ",".join(cols)


def row(k: int, rows: int, err: float, end_mode: str, flags: set[str]) -> str:
    mode = "tracking" if k < rows // 2 else end_mode
    q = [0.1 * j for j in range(7)]
    qd = [0.0] * 7
    if "moving" in flags and k >= rows - 50:
        qd[0] = 0.05
    tau = [1.0] * 7
    if "overtau" in flags and k == 0:
        tau[0] = 1000.0
    p = [PD[0] + err, PD[1], PD[2]]
    vals = [k * DT_NS, mode, *q, *qd, *q, *[0.0] * 7, *tau, *p, *[0.0] * 3]
    vals += [*PD, *[0.0] * 3, *[0.0] * 6, 0]
    return ",".join(str(v) for v in vals)


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
    source = args.get("command_source", "")
    marker = Path(str(log) + ".launch")
    Path(str(log) + ".echo").unlink(missing_ok=True)
    marker.write_text(source or "config", encoding="utf-8")
    end_mode = "finished"
    if source == "external":
        end_mode = "holding" if "holdend" in flags else "standby"
    t_end_ns = round(float(os.environ.get("FAKE_ROS2_T_END_S", "1.0")) * 1e9)
    rows = t_end_ns // DT_NS + 1
    err = 0.01 if "fail" in flags else 0.004 if "pos4mm" in flags else 0.0
    csv = None
    if "nocsv" not in flags and "log_path" in args:
        path = Path(args["log_path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        csv = path.open("w", encoding="utf-8")
        csv.write(header() + "\n")
        csv.writelines(row(k, rows, TRANSIENT_M, end_mode, flags) + "\n" for k in range(rows // 2))
        csv.flush()
    deadline = time.monotonic() + float(os.environ.get("FAKE_ROS2_MAX_S", "120"))
    while not stop and time.monotonic() < deadline:
        time.sleep(0.05)
    marker.unlink(missing_ok=True)
    if not stop:
        return 4
    if csv is not None:
        csv.writelines(row(k, rows, err, end_mode, flags) + "\n" for k in range(rows // 2, rows))
        csv.close()
    append(log, {"signal": stop[0]})
    return 0


def next_status(log: Path, flags: set[str]) -> str:
    source = Path(str(log) + ".launch").read_text(encoding="utf-8")
    counter = Path(str(log) + ".echo")
    count = int(counter.read_text(encoding="utf-8")) if counter.exists() else 0
    counter.write_text(str(count + 1), encoding="utf-8")
    if "nostandby" in flags or count < int(os.environ.get("FAKE_ROS2_STANDBY_AFTER", "2")):
        return "idle"
    return "standby" if source == "external" else "tracking"


def echo(argv: list[str], flags: set[str], log: Path) -> int:
    if not Path(str(log) + ".launch").exists():
        time.sleep(0.2)
        append(log, {"status": []})
        return 1
    if STATUS_TOPIC not in argv:
        return 0
    field = "--field" in argv
    printed = []
    for _ in range(1 if "--once" in argv else 50):
        mode = next_status(log, flags)
        printed.append(mode)
        print(mode if field else f"data: {mode}")
        print("---", flush=True)
        if mode == "standby":
            break
    append(log, {"status": printed})
    return 0


def client(argv: list[str], log: Path) -> int:
    running = Path(str(log) + ".launch").exists()
    append(log, {"client": argv, "launch_running": running})
    default = "3" if "--cancel-after" in argv else "0"
    code = int(os.environ.get("FAKE_CLIENT_EXIT", default))
    print("status: " + {0: "succeeded", 3: "canceled"}.get(code, "aborted"))
    print(f"success: {'true' if code == 0 else 'false'}")
    return code


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_ROS2_MODE", "").split(",")))
    log = Path(os.environ["FAKE_ROS2_LOG"])
    argv = sys.argv[1:]
    append(log, {"argv": argv, "pid": os.getpid()})
    if argv[:1] == ["launch"]:
        return launch(argv, flags, log)
    if argv[:2] == ["topic", "echo"]:
        return echo(argv, flags, log)
    if argv[:1] == ["run"]:
        return client(argv[1:], log)
    if argv[:2] == ["control", "list_controllers"]:
        print("joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active")
        print("cobot_controller        cobot_ros2/CobotController                    active")
    return 0


if __name__ == "__main__":
    sys.exit(main())
