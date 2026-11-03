"""Stand-in for the `ros2` CLI used by test_run_e2e_gazebo.py; stdlib only.

`ros2 launch ...` logs {"argv", "pid"} as a JSON line to FAKE_ROS2_LOG, then behaves like the
launched controller's CSV writer: it writes the header and the first half of a Run CSV to the
`log_path:=` argument at once, waits for SIGINT or SIGTERM, then appends the rest of the rows,
logs {"signal"} and exits 0. Without a signal it gives up after FAKE_ROS2_MAX_S seconds
(default 120) with exit 4 and leaves the CSV half written. While a launch runs it keeps a marker
file (FAKE_ROS2_LOG + ".launch").

Rows: t_ns = k * 1 ms up to FAKE_ROS2_T_END_S (default 1.0 s). First half: tip error 50 mm
(transient, so a verdict taken before the launch is stopped fails). Second half: tip error set
by FAKE_ROS2_MODE: empty = 0, pos4mm = 4 mm, fail = 10 mm. Other modes: nocsv = never writes
a CSV (and leaves no black-box region); crash = exit 3 at once; shortcsv = the second half is
never written to log_path (the controller died: the CSV stays short, the region holds every row);
csvlost = no CSV at log_path, the region holds every row. q = qref everywhere, tau = 1 N*m.

Black-box log (feature blackbox-log):
`ros2 service call /cobot_controller/dump_log std_srvs/srv/Trigger` is logged as
{"dump_log": argv after `call`, "launch_running": bool} and prints a Trigger response; flag
dumpfail = exit 1 with success=False, dumphang = no answer for 60 s.
`ros2 run cobot_ros2 cobot_logdump <controller> --out CSV` is logged as {"logdump": argv after
`run`}; it writes the whole Run CSV of the launch (header and every row, the second half as
above) to CSV, prints `<rows> rows -> CSV` and exits 0; with nocsv, crash or noregion it prints
the reason on stderr and exits 1 (no region).

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
    cols += [f"e{k}" for k in range(1, 7)] + ["step_us", "hold"]
    return ",".join(cols)


def row(k: int, rows: int, err: float) -> str:
    mode = "tracking" if k < rows // 2 else "finished"
    q = [0.1 * j for j in range(7)]
    p = [PD[0] + err, PD[1], PD[2]]
    vals = [k * DT_NS, mode, *q, *[0.0] * 7, *q, *[0.0] * 7, *[1.0] * 7, *p, *[0.0] * 3]
    vals += [*PD, *[0.0] * 3, *[0.0] * 6, 0, "none"]
    return ",".join(str(v) for v in vals)


def row_count() -> int:
    t_end_ns = round(float(os.environ.get("FAKE_ROS2_T_END_S", "1.0")) * 1e9)
    return t_end_ns // DT_NS + 1


def final_err(flags: set[str]) -> float:
    return 0.01 if "fail" in flags else 0.004 if "pos4mm" in flags else 0.0


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
    rows = row_count()
    err = final_err(flags)
    marker = Path(str(log) + ".launch")
    marker.write_text("running", encoding="utf-8")
    csv = None
    if not flags & {"nocsv", "csvlost"} and "log_path" in args:
        path = Path(args["log_path"])
        path.parent.mkdir(parents=True, exist_ok=True)
        csv = path.open("w", encoding="utf-8")
        csv.write(header() + "\n")
        csv.writelines(row(k, rows, TRANSIENT_M) + "\n" for k in range(rows // 2))
        csv.flush()
    deadline = time.monotonic() + float(os.environ.get("FAKE_ROS2_MAX_S", "120"))
    while not stop and time.monotonic() < deadline:
        time.sleep(0.05)
    marker.unlink(missing_ok=True)
    if not stop:
        return 4
    if csv is not None:
        if "shortcsv" not in flags:
            csv.writelines(row(k, rows, err) + "\n" for k in range(rows // 2, rows))
        csv.close()
    append(log, {"signal": stop[0]})
    return 0


def dump_log(argv: list[str], flags: set[str], log: Path) -> int:
    append(log, {"dump_log": argv, "launch_running": Path(str(log) + ".launch").exists()})
    print("requester: making request: std_srvs.srv.Trigger_Request()\n\nresponse:", flush=True)
    if "dumphang" in flags:
        time.sleep(60)
    if "dumpfail" in flags:
        print("std_srvs.srv.Trigger_Response(success=False, message='no recorder')")
        return 1
    print("std_srvs.srv.Trigger_Response(success=True, message='run.csv 0 rows')")
    return 0


def logdump(argv: list[str], flags: set[str], log: Path) -> int:
    append(log, {"logdump": argv})
    if flags & {"nocsv", "crash", "noregion"}:
        print("cobot_logdump: no region /cobot_log_cobot_controller", file=sys.stderr)
        return 1
    out = Path(argv[argv.index("--out") + 1])
    rows = row_count()
    err = final_err(flags)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as f:
        f.write(header() + "\n")
        f.writelines(row(k, rows, TRANSIENT_M) + "\n" for k in range(rows // 2))
        f.writelines(row(k, rows, err) + "\n" for k in range(rows // 2, rows))
    print(f"{rows} rows -> {out}")
    return 0


def main() -> int:
    flags = set(filter(None, os.environ.get("FAKE_ROS2_MODE", "").split(",")))
    log = Path(os.environ["FAKE_ROS2_LOG"])
    argv = sys.argv[1:]
    append(log, {"argv": argv, "pid": os.getpid()})
    if argv[:1] == ["launch"]:
        return launch(argv, flags, log)
    if argv[:2] == ["service", "call"]:
        return dump_log(argv[2:], flags, log)
    if argv[:3] == ["run", "cobot_ros2", "cobot_logdump"]:
        return logdump(argv[1:], flags, log)
    if argv[:2] == ["control", "list_controllers"]:
        print("joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster active")
        print("cobot_controller        cobot_ros2/CobotController                    active")
    return 0


if __name__ == "__main__":
    sys.exit(main())
