"""tools/send_waypoints.py, the FollowWaypoints / PreviewWaypoints client (feature waypoint-ros),
through its pure functions (no ROS: message modules are replaced by fakes).

Contract pinned here (module importable without rclpy, cobot_interfaces or geometry_msgs; the
ROS imports happen inside the functions that need them):
- parse_args(argv) -> Namespace(file: Path, cancel_after: float | None, preview: bool);
  --cancel-after must be > 0 (argparse error, exit 2, otherwise).
- rpy_deg_to_quaternion(rpy_deg) -> (w, x, y, z), R = Rz(yaw) Ry(pitch) Rx(roll) as everywhere in
  the repo (docs/conventions.md: quaternions (w, x, y, z)).
- read_waypoints(path) -> list of {"position_m": [x, y, z], "quaternion_wxyz": (w, x, y, z),
  "duration_s": float, "stop": bool} from the waypoint file (docs/conventions.md, design 6.6):
  frame must be base_link, duration_s and stop optional (0.0, False); ValueError naming
  `frame`, `waypoints` or `waypoints[i].<key>` otherwise.
- build_goal(waypoints) -> cobot_interfaces.action.FollowWaypoints.Goal and
  build_preview_request(waypoints) -> cobot_interfaces.srv.PreviewWaypoints.Request, one
  cobot_interfaces.msg.Waypoint per entry (TCP pose as given, geometry_msgs/Pose).
- result_to_dict(result) and preview_to_dict(response): plain Python values (yaml.safe_dump-able).
- feedback_line(feedback) -> one line naming the segment and the mode.
- exit_code(status, success, cancel_requested): 0 succeeded with success, 3 canceled when the
  cancel was requested (--cancel-after), 1 otherwise (aborted, rejected, unrequested cancel).
"""

import array
import importlib.util
import math
import re
import sys
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CLIENT = REPO / "tools/send_waypoints.py"
ROS_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.node",
    "cobot_interfaces",
    "cobot_interfaces.msg",
    "cobot_interfaces.action",
    "cobot_interfaces.srv",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
)
SQRT_HALF = math.sqrt(0.5)  # cos 45 deg = sin 45 deg = 0.7071067811865476


# ---------------------------------------------------------------- fake ROS messages


class FakeMsg:
    """ROS-message-like: fields with defaults, settable by keyword or attribute."""

    FIELDS: dict = {}

    def __init__(self, **kwargs):
        for name, default in self.FIELDS.items():
            setattr(self, name, default() if callable(default) else default)
        for name, value in kwargs.items():
            assert name in self.FIELDS, f"{type(self).__name__} has no field {name}"
            setattr(self, name, value)


def msg_class(name, fields):
    return type(name, (FakeMsg,), {"FIELDS": fields})


Point = msg_class("Point", {"x": 0.0, "y": 0.0, "z": 0.0})
Quaternion = msg_class("Quaternion", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
Pose = msg_class("Pose", {"position": Point, "orientation": Quaternion})
Waypoint = msg_class("Waypoint", {"pose": Pose, "duration_s": 0.0, "stop": False})
WaypointIssue = msg_class(
    "WaypointIssue", {"waypoint": 0, "issue": "", "joint": 0, "value": 0.0, "limit": 0.0}
)
Path_ = msg_class("Path", {"poses": list})


class FollowWaypoints:
    Goal = msg_class("Goal", {"waypoints": list})
    Result = msg_class(
        "Result",
        {
            "success": False,
            "message": "",
            "issues": list,
            "final_pos_err_m": 0.0,
            "final_rot_err_deg": 0.0,
            "rms_joint_err_rad": 0.0,
            "peak_tau_ratio": 0.0,
        },
    )
    Feedback = msg_class(
        "Feedback",
        {
            "segment": 0,
            "t_s": 0.0,
            "duration_s": 0.0,
            "pos_err_m": 0.0,
            "rot_err_deg": 0.0,
            "mode": "",
        },
    )


class PreviewWaypoints:
    Request = msg_class("Request", {"waypoints": list})
    Response = msg_class(
        "Response",
        {
            "ok": False,
            "issues": list,
            "path": Path_,
            "segment_durations_s": lambda: array.array("d"),
        },
    )


def fake_ros_modules():
    mods = {name: types.ModuleType(name) for name in ROS_MODULES if not name.startswith("rclpy")}
    for name, mod in mods.items():
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(mods[parent], child, mod)
    mods["geometry_msgs.msg"].Point = Point
    mods["geometry_msgs.msg"].Quaternion = Quaternion
    mods["geometry_msgs.msg"].Pose = Pose
    mods["nav_msgs.msg"].Path = Path_
    mods["cobot_interfaces.msg"].Waypoint = Waypoint
    mods["cobot_interfaces.msg"].WaypointIssue = WaypointIssue
    mods["cobot_interfaces.action"].FollowWaypoints = FollowWaypoints
    mods["cobot_interfaces.srv"].PreviewWaypoints = PreviewWaypoints
    return mods


def load_client():
    spec = importlib.util.spec_from_file_location("send_waypoints_under_test", CLIENT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def no_ros(monkeypatch):
    """The client module with every ROS module made unimportable."""
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, None)
    return load_client()


@pytest.fixture
def with_fake_ros(monkeypatch):
    """The client module with fake message modules (rclpy still unimportable)."""
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, None)
    for name, mod in fake_ros_modules().items():
        monkeypatch.setitem(sys.modules, name, mod)
    return load_client()


# ---------------------------------------------------------------- helpers


def quat_matrix(q):
    """Rotation matrix of a unit quaternion (w, x, y, z), Hamilton convention."""
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def rpy_matrix(rpy_deg):
    """R = Rz(yaw) Ry(pitch) Rx(roll), built independently of the client."""
    r, p, y = np.radians(rpy_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def same_rotation(q, expected):
    """q equals expected (w, x, y, z) up to the sign of the whole quaternion."""
    q = np.asarray(q, dtype=float)
    expected = np.asarray(expected, dtype=float)
    return np.allclose(q, expected, atol=1e-12) or np.allclose(q, -expected, atol=1e-12)


def write_file(tmp_path, data):
    path = tmp_path / "wp.yaml"
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    return path


TWO_POINTS = {
    "frame": "base_link",
    "waypoints": [
        {"position_m": [-0.5, -0.6, 0.9], "rpy_deg": [0, 0, 90]},
        {"position_m": [-0.3, -0.6, 0.7], "rpy_deg": [90, 0, 0], "duration_s": 4.0, "stop": True},
    ],
}


# ---------------------------------------------------------------- import and arguments


def test_importable_without_ros(no_ros):
    for name in ("parse_args", "read_waypoints", "rpy_deg_to_quaternion", "build_goal"):
        assert callable(getattr(no_ros, name)), name
    for name in ("build_preview_request", "result_to_dict", "preview_to_dict"):
        assert callable(getattr(no_ros, name)), name
    assert callable(no_ros.feedback_line) and callable(no_ros.exit_code)


def test_parse_args(no_ros):
    args = no_ros.parse_args(["wp.yaml"])
    assert args.file == Path("wp.yaml")
    assert args.cancel_after is None and args.preview is False
    args = no_ros.parse_args(["wp.yaml", "--cancel-after", "3"])
    assert args.cancel_after == 3.0
    assert no_ros.parse_args(["wp.yaml", "--preview"]).preview is True


@pytest.mark.parametrize(
    "argv", [[], ["wp.yaml", "--cancel-after", "0"], ["wp.yaml", "--cancel-after", "-1"]]
)
def test_parse_args_rejects(no_ros, argv):
    with pytest.raises(SystemExit) as err:
        no_ros.parse_args(argv)
    assert err.value.code == 2


# ---------------------------------------------------------------- rpy -> quaternion


@pytest.mark.parametrize(
    ("rpy_deg", "expected"),
    # Half angles: 90 deg about one axis -> w = cos 45 deg, that axis = sin 45 deg (0.70710678).
    [
        ((0, 0, 0), (1.0, 0.0, 0.0, 0.0)),
        ((0, 0, 90), (SQRT_HALF, 0.0, 0.0, SQRT_HALF)),
        ((90, 0, 0), (SQRT_HALF, SQRT_HALF, 0.0, 0.0)),
        ((0, 90, 0), (SQRT_HALF, 0.0, SQRT_HALF, 0.0)),
        ((0, 0, 180), (0.0, 0.0, 0.0, 1.0)),
    ],
)
def test_rpy_single_axis(no_ros, rpy_deg, expected):
    q = no_ros.rpy_deg_to_quaternion(rpy_deg)
    assert len(q) == 4
    assert same_rotation(q, expected), q


@pytest.mark.parametrize("rpy_deg", [(30, 45, 60), (-120, 10, 170), (5, -80, -35)])
def test_rpy_order_is_z_y_x(no_ros, rpy_deg):
    q = np.asarray(no_ros.rpy_deg_to_quaternion(rpy_deg), dtype=float)
    assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-12)
    assert np.allclose(quat_matrix(q), rpy_matrix(rpy_deg), atol=1e-12)


# ---------------------------------------------------------------- waypoint file


def test_read_waypoints(no_ros, tmp_path):
    wps = no_ros.read_waypoints(write_file(tmp_path, TWO_POINTS))
    assert len(wps) == 2
    assert list(wps[0]["position_m"]) == [-0.5, -0.6, 0.9]
    assert same_rotation(wps[0]["quaternion_wxyz"], (SQRT_HALF, 0.0, 0.0, SQRT_HALF))
    assert wps[0]["duration_s"] == 0.0 and wps[0]["stop"] is False  # optional keys
    assert list(wps[1]["position_m"]) == [-0.3, -0.6, 0.7]
    assert same_rotation(wps[1]["quaternion_wxyz"], (SQRT_HALF, SQRT_HALF, 0.0, 0.0))
    assert wps[1]["duration_s"] == 4.0 and wps[1]["stop"] is True


def bad(key_path, mutate):
    data = yaml.safe_load(yaml.safe_dump(TWO_POINTS))
    mutate(data)
    return pytest.param(data, key_path, id=key_path)


@pytest.mark.parametrize(
    ("data", "named"),
    [
        bad("frame", lambda d: d.update(frame="world")),
        bad("waypoints", lambda d: d.update(waypoints=[])),
        bad("waypoints[1].position_m", lambda d: d["waypoints"][1].update(position_m=[0.1, 0.2])),
        bad("waypoints[1].position_m", lambda d: d["waypoints"][1].pop("position_m")),
        bad("waypoints[0].rpy_deg", lambda d: d["waypoints"][0].update(rpy_deg=[0, 0])),
        bad("waypoints[0].rpy_deg", lambda d: d["waypoints"][0].pop("rpy_deg")),
        bad("waypoints[0].duration_s", lambda d: d["waypoints"][0].update(duration_s="abc")),
        bad("waypoints[1].stop", lambda d: d["waypoints"][1].update(stop=3)),
    ],
)
def test_read_waypoints_names_the_bad_key(no_ros, tmp_path, data, named):
    with pytest.raises(ValueError, match=re.escape(named)):
        no_ros.read_waypoints(write_file(tmp_path, data))


# ---------------------------------------------------------------- goal and preview request


def check_waypoint_msgs(msgs, wps):
    assert len(msgs) == len(wps)
    for msg, wp in zip(msgs, wps, strict=True):
        assert isinstance(msg, Waypoint)
        p, o = msg.pose.position, msg.pose.orientation
        assert (p.x, p.y, p.z) == tuple(wp["position_m"])
        assert (o.w, o.x, o.y, o.z) == pytest.approx(tuple(wp["quaternion_wxyz"]), abs=1e-15)
        assert msg.duration_s == wp["duration_s"]
        assert msg.stop is wp["stop"]


def test_build_goal(with_fake_ros, tmp_path):
    wps = with_fake_ros.read_waypoints(write_file(tmp_path, TWO_POINTS))
    goal = with_fake_ros.build_goal(wps)
    assert isinstance(goal, FollowWaypoints.Goal)
    check_waypoint_msgs(list(goal.waypoints), wps)
    o = goal.waypoints[0].pose.orientation  # rpy (0, 0, 90): w = z = 0.70710678
    assert abs(o.w) == pytest.approx(SQRT_HALF, abs=1e-12)
    assert abs(o.z) == pytest.approx(SQRT_HALF, abs=1e-12)


def test_build_preview_request(with_fake_ros, tmp_path):
    wps = with_fake_ros.read_waypoints(write_file(tmp_path, TWO_POINTS))
    request = with_fake_ros.build_preview_request(wps)
    assert isinstance(request, PreviewWaypoints.Request)
    check_waypoint_msgs(list(request.waypoints), wps)


# ---------------------------------------------------------------- printing and exit code


def test_result_to_dict_is_plain_yaml(with_fake_ros):
    issue = WaypointIssue(waypoint=2, issue="torque", joint=1, value=0.95, limit=0.9)
    result = FollowWaypoints.Result(
        success=False,
        message="validation failed",
        issues=[issue],
        final_pos_err_m=0.001,
        final_rot_err_deg=0.2,
        rms_joint_err_rad=0.003,
        peak_tau_ratio=0.7,
    )
    d = with_fake_ros.result_to_dict(result)
    assert d["success"] is False and d["message"] == "validation failed"
    assert d["issues"] == [
        {"waypoint": 2, "issue": "torque", "joint": 1, "value": 0.95, "limit": 0.9}
    ]
    assert d["final_pos_err_m"] == 0.001 and d["final_rot_err_deg"] == 0.2
    assert d["rms_joint_err_rad"] == 0.003 and d["peak_tau_ratio"] == 0.7
    assert yaml.safe_load(yaml.safe_dump(d)) == d


def test_preview_to_dict_is_plain_yaml(with_fake_ros):
    response = PreviewWaypoints.Response(ok=True)
    response.segment_durations_s = array.array("d", [2.5, 4.0])  # as rclpy delivers float64[]
    d = with_fake_ros.preview_to_dict(response)
    assert d["ok"] is True and d["issues"] == []
    assert d["segment_durations_s"] == [2.5, 4.0]
    assert yaml.safe_load(yaml.safe_dump(d)) == d


def test_feedback_line(with_fake_ros):
    fb = FollowWaypoints.Feedback(
        segment=3, t_s=1.25, duration_s=6.0, pos_err_m=0.0004, rot_err_deg=0.1, mode="tracking"
    )
    line = with_fake_ros.feedback_line(fb)
    assert "\n" not in line.rstrip("\n")
    assert "3" in line and "tracking" in line


@pytest.mark.parametrize(
    ("status", "success", "cancel_requested", "code"),
    [
        ("succeeded", True, False, 0),
        ("succeeded", False, False, 1),
        ("aborted", False, False, 1),
        ("rejected", False, False, 1),
        ("canceled", False, True, 3),
        ("canceled", False, False, 1),  # canceled by someone else
    ],
)
def test_exit_code(no_ros, status, success, cancel_requested, code):
    assert no_ros.exit_code(status, success, cancel_requested) == code
