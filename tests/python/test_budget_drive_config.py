"""The budget controller config with the drive-side position loop (feature drive-loop, ADR-023).

config/budget_xarm7_drive.yaml equals config/budget_xarm7.yaml except cobot.control.algorithm
(drive_position), the cobot.drive block and cobot.safety.hal_timeout_ms (20 ms = 5 servo periods at
250 Hz, owner question C2: with 5 ms one lost packet or one 5-cycle latency burst already trips
state_timeout whatever the law). The header comment gives that reason; budget_xarm7.yaml keeps
computed_torque (E2E-12 finding) and points to the drive file in its comments.
"""

import copy
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
BASE = REPO / "config/budget_xarm7.yaml"
DRIVE = REPO / "config/budget_xarm7_drive.yaml"
DRIVE_KEYS = {"rate_hz", "omega_rad_s", "zeta", "velocity_filter_hz", "torque_feedforward"}


def load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_drive_file_equals_budget_except_the_drive_keys():
    base, drive = load(BASE), load(DRIVE)
    a, b = copy.deepcopy(base), copy.deepcopy(drive)
    del a["cobot"]["control"]["algorithm"]
    del b["cobot"]["control"]["algorithm"]
    del a["cobot"]["safety"]["hal_timeout_ms"]
    del b["cobot"]["safety"]["hal_timeout_ms"]
    assert "drive" not in a["cobot"]
    del b["cobot"]["drive"]
    assert a == b, "budget_xarm7_drive.yaml differs from budget_xarm7.yaml beyond the drive keys"


def test_drive_values():
    cobot = load(DRIVE)["cobot"]
    assert cobot["control"] == {"algorithm": "drive_position"}
    assert cobot["safety"]["hal_timeout_ms"] == 20.0
    drive = cobot["drive"]
    assert set(drive) == DRIVE_KEYS
    assert drive["rate_hz"] == 4000.0
    assert drive["torque_feedforward"] == "inverse_dynamics"  # owner question C1 default
    omega = drive["omega_rad_s"]
    values = omega if isinstance(omega, list) else [omega]
    assert len(values) in (1, 7)
    assert all(20.0 <= w <= 150.0 for w in values), omega  # the swept range
    assert drive["zeta"] > 0 and drive["velocity_filter_hz"] > 0


def test_budget_base_unchanged_and_points_to_the_drive_file():
    base = load(BASE)["cobot"]
    assert base["control"] == {"algorithm": "computed_torque"}
    assert base["safety"]["hal_timeout_ms"] == 5.0
    assert "config/budget_xarm7_drive.yaml" in BASE.read_text(encoding="utf-8")


def test_header_comment_gives_the_hal_timeout_reason():
    lines = DRIVE.read_text(encoding="utf-8").splitlines()
    header = []
    for line in lines:
        if not line.startswith("#"):
            break
        header.append(line)
    text = " ".join(header)
    assert "hal_timeout_ms" in text and "20" in text, header
    assert "drive_position" in text, header
