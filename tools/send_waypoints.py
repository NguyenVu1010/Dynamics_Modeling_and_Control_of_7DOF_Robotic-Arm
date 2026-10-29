#!/usr/bin/env python3
"""Client of the controller's external command mode (design 2026-10-29 section 6.4).

Usage: send_waypoints.py FILE [--cancel-after S] [--preview]
FILE is a waypoint file (docs/conventions.md "Waypoint file": TCP poses in base_link). Without
--preview it sends FILE as a FollowWaypoints goal to /cobot_controller/follow_waypoints, prints one
line per feedback message and the result as YAML; --cancel-after S cancels the goal S seconds
after it was accepted. With --preview it calls /cobot_controller/preview_waypoints and prints the
response as YAML. Exit 0 goal succeeded (or preview ok), 3 canceled as requested, 1 otherwise
(rejected, aborted, unrequested cancel, preview not ok, bad file, no server).

The pure functions import no ROS module at load time, so they are testable without ROS.
"""

import argparse
import math
import sys
import time
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any

import yaml

ACTION = "/cobot_controller/follow_waypoints"
PREVIEW = "/cobot_controller/preview_waypoints"
SERVER_WAIT_S = 30.0
SPIN_S = 0.05
STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}  # action_msgs/GoalStatus values


def positive(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError(f"must be > 0, got {text}")
    return value


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="send_waypoints", description=__doc__)
    parser.add_argument("file", type=Path)
    parser.add_argument("--cancel-after", type=positive, default=None, metavar="S")
    parser.add_argument("--preview", action="store_true")
    return parser.parse_args(argv)


def rpy_deg_to_quaternion(rpy_deg: Sequence[float]) -> tuple[float, float, float, float]:
    """(w, x, y, z) of R = Rz(yaw) Ry(pitch) Rx(roll), Hamilton convention."""
    r, p, y = (math.radians(float(a)) / 2.0 for a in rpy_deg)
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _number(value: object) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool) and math.isfinite(value)


def _triple(entry: dict, key: str, where: str) -> list[float]:
    value = entry.get(key)
    if not isinstance(value, list) or len(value) != 3 or not all(map(_number, value)):
        raise ValueError(f"{where}.{key}: expected 3 finite numbers, got {value!r}")
    return [float(v) for v in value]


def read_waypoints(path: Path) -> list[dict]:
    """Waypoints of the file as {position_m, quaternion_wxyz, duration_s, stop}."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict) or data.get("frame") != "base_link":
        raise ValueError(f"{path}: frame: expected base_link")
    entries = data.get("waypoints")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f"{path}: waypoints: expected a non-empty list")
    out = []
    for i, entry in enumerate(entries):
        where = f"{path}: waypoints[{i}]"
        if not isinstance(entry, dict):
            raise ValueError(f"{where}: expected a mapping")
        duration = entry.get("duration_s", 0.0)
        if not _number(duration) or duration < 0.0:
            raise ValueError(f"{where}.duration_s: expected a number >= 0, got {duration!r}")
        stop = entry.get("stop", False)
        if not isinstance(stop, bool):
            raise ValueError(f"{where}.stop: expected true or false, got {stop!r}")
        out.append(
            {
                "position_m": _triple(entry, "position_m", where),
                "quaternion_wxyz": rpy_deg_to_quaternion(_triple(entry, "rpy_deg", where)),
                "duration_s": float(duration),
                "stop": stop,
            }
        )
    return out


def _waypoint_msgs(waypoints: list[dict]) -> list:
    from cobot_interfaces.msg import Waypoint  # noqa: PLC0415
    from geometry_msgs.msg import Point, Pose, Quaternion  # noqa: PLC0415

    msgs = []
    for wp in waypoints:
        x, y, z = wp["position_m"]
        qw, qx, qy, qz = wp["quaternion_wxyz"]
        pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
        msgs.append(Waypoint(pose=pose, duration_s=wp["duration_s"], stop=wp["stop"]))
    return msgs


def build_goal(waypoints: list[dict]) -> Any:
    from cobot_interfaces.action import FollowWaypoints  # noqa: PLC0415

    return FollowWaypoints.Goal(waypoints=_waypoint_msgs(waypoints))


def build_preview_request(waypoints: list[dict]) -> Any:
    from cobot_interfaces.srv import PreviewWaypoints  # noqa: PLC0415

    return PreviewWaypoints.Request(waypoints=_waypoint_msgs(waypoints))


def _issues(issues: Iterable[Any]) -> list[dict]:
    keys = ("waypoint", "issue", "joint", "value", "limit")
    types = (int, str, int, float, float)
    return [{k: t(getattr(i, k)) for k, t in zip(keys, types, strict=True)} for i in issues]


def result_to_dict(result: Any) -> dict:
    out = {"success": bool(result.success), "message": str(result.message)}
    out["issues"] = _issues(result.issues)
    for key in ("final_pos_err_m", "final_rot_err_deg", "rms_joint_err_rad", "peak_tau_ratio"):
        out[key] = float(getattr(result, key))
    return out


def preview_to_dict(response: Any) -> dict:
    return {
        "ok": bool(response.ok),
        "issues": _issues(response.issues),
        "path_poses": len(response.path.poses),
        "segment_durations_s": [float(d) for d in response.segment_durations_s],
    }


def feedback_line(feedback: Any) -> str:
    return (
        f"segment {feedback.segment} t {feedback.t_s:.2f}/{feedback.duration_s:.2f} s "
        f"pos_err {feedback.pos_err_m * 1e3:.3f} mm rot_err {feedback.rot_err_deg:.3f} deg "
        f"mode {feedback.mode}"
    )


def exit_code(status: str, success: bool, cancel_requested: bool) -> int:
    if status == "succeeded" and success:
        return 0
    return 3 if status == "canceled" and cancel_requested else 1


def _spin_until(node: Any, future: Any, rclpy: Any, deadline: float | None = None) -> bool:
    while not future.done():
        if deadline is not None and time.monotonic() > deadline:
            return False
        rclpy.spin_once(node, timeout_sec=SPIN_S)
    return True


def run_preview(node: Any, rclpy: Any, waypoints: list[dict]) -> int:
    from cobot_interfaces.srv import PreviewWaypoints  # noqa: PLC0415

    client = node.create_client(PreviewWaypoints, PREVIEW)
    if not client.wait_for_service(timeout_sec=SERVER_WAIT_S):
        print(f"error: no service {PREVIEW}", file=sys.stderr)
        return 1
    future = client.call_async(build_preview_request(waypoints))
    _spin_until(node, future, rclpy)
    response = preview_to_dict(future.result())
    print(yaml.safe_dump(response, sort_keys=False), end="")
    return 0 if response["ok"] else 1


def run_goal(node: Any, rclpy: Any, waypoints: list[dict], cancel_after: float | None) -> int:
    from cobot_interfaces.action import FollowWaypoints  # noqa: PLC0415
    from rclpy.action import ActionClient  # noqa: PLC0415

    client = ActionClient(node, FollowWaypoints, ACTION)
    if not client.wait_for_server(timeout_sec=SERVER_WAIT_S):
        print(f"error: no action server {ACTION}", file=sys.stderr)
        return 1

    def on_feedback(msg: Any) -> None:
        print(feedback_line(msg.feedback), flush=True)

    sent = client.send_goal_async(build_goal(waypoints), feedback_callback=on_feedback)
    _spin_until(node, sent, rclpy)
    handle = sent.result()
    if not handle.accepted:
        print("status: rejected")
        return exit_code("rejected", False, False)
    result = handle.get_result_async()
    cancel_requested = False
    if cancel_after is not None and not _spin_until(
        node, result, rclpy, time.monotonic() + cancel_after
    ):
        handle.cancel_goal_async()
        cancel_requested = True
    _spin_until(node, result, rclpy)
    status = STATUS.get(result.result().status, "unknown")
    outcome = result.result().result
    print(yaml.safe_dump({"status": status, **result_to_dict(outcome)}, sort_keys=False), end="")
    return exit_code(status, outcome.success, cancel_requested)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        waypoints = read_waypoints(args.file)
    except (OSError, ValueError, yaml.YAMLError) as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    import rclpy  # noqa: PLC0415
    from rclpy.node import Node  # noqa: PLC0415

    rclpy.init()
    node = Node("send_waypoints")
    try:
        if args.preview:
            return run_preview(node, rclpy, waypoints)
        return run_goal(node, rclpy, waypoints, args.cancel_after)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
