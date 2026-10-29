"""Stand-in for the `ros2` CLI used by test_run_e2e_gazebo.py; stdlib only.

`ros2 launch ...` logs {"argv", "pid"} as a JSON line to FAKE_ROS2_LOG, then behaves like the
launched controller's CSV writer: it writes the header and the first half of a Run CSV to the
`log_path:=` argument at once, waits for SIGINT or SIGTERM, then appends the rest of the rows,
logs {"signal"} and exits 0. Without a signal it gives up after FAKE_ROS2_MAX_S seconds
(default 120) with exit 4 and leaves the CSV half written.

Rows: t_ns = k * 1 ms up to FAKE_ROS2_T_END_S (default 1.0 s). First half: tip error 50 mm
(transient, so a verdict taken before the launch is stopped fails). Second half: tip error set
by FAKE_ROS2_MODE: empty = 0, pos4mm = 4 mm, fail = 10 mm. Other modes: nocsv = never writes
a CSV; crash = exit 3 at once. q = qref everywhere, tau = 1 N*m.

Any other subcommand (for example `ros2 control list_controllers`) is logged, reports
cobot_controller as active and exits 0.
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


def header() -> str:
    cols = ["t_ns", "mode"]
    cols += [f"{p}{j}" for p in ("q", "qd", "qref", "qdref", "tau") for j in range(1, 8)]
    cols += ["px", "py", "pz", "rx", "ry", "rz", "pdx", "pdy", "pdz", "rdx", "rdy", "rdz"]
    cols += [f"e{k}" for k in range(1, 7)] + ["step_us"]
    return ",".join(cols)


def row(k: int, rows: int, err: float) -> str:
    mode = "tracking" if k < rows // 2 else "finished"
    q = [0.1 * j for j in range(7)]
    p = [PD[0] + err, PD[1], PD[2]]
    vals = [k * DT_NS, mode, *q, *[0.0] * 7, *q, *[0.0] * 7, *[1.0] * 7, *p, *[0.0] * 3]
    vals += [*PD, *[0.0] * 3, *[0.0] * 6, 0]
    return ",".join(str(v) for v in vals)


def launch(argv: list[str], flags: set[str], log: Path) -> int:
    stop: list[int] = []

    def on_signal(signum: int, _frame: object) -> None:
        stop.append(signum)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if "crash" in flags:
        return 3
    args = dict(a.split(":=", 1) for a in argv if ":=" in a)
    t_end_ns = round(float(os.environ.get("FAKE_ROS2_T_END_S", "1.0")) * 1e9)
    rows = t_end_ns // DT_NS + 1
    err = 0.01 if "fail" in flags else 0.004 if "pos4mm" in flags else 0.0
    csv = None
    if "nocsv" not in flags and "log_path" in args:
        path = Path(args["log_path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        csv = path.open("w", encoding="utf-8")
        csv.write(header() + "\n")
        csv.writelines(row(k, rows, TRANSIENT_M) + "\n" for k in range(rows // 2))
        csv.flush()
    deadline = time.monotonic() + float(os.environ.get("FAKE_ROS2_MAX_S", "120"))
    while not stop and time.monotonic() < deadline:
        time.sleep(0.05)
    if not stop:
        return 4
    if csv is not None:
        csv.writelines(row(k, rows, err) + "\n" for k in range(rows // 2, rows))
        csv.close()
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps({"signal": stop[0]}) + "\n")
    return 0


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_ROS2_MODE", "").split(",")))
    log = Path(os.environ["FAKE_ROS2_LOG"])
    argv = sys.argv[1:]
    with log.open("a", encoding="utf-8") as f:
        f.write(json.dumps({"argv": argv, "pid": os.getpid()}) + "\n")
    if argv[:1] == ["launch"]:
        return launch(argv, flags, log)
    if argv[:2] == ["control", "list_controllers"]:
        print("joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active")
        print("cobot_controller        cobot_ros2/CobotController                    active")
    return 0


if __name__ == "__main__":
    sys.exit(main())
