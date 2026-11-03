"""Controller config per reference profile (owner decision 2026-11-03, feature profiles).

config/<profile>.yaml equals config/default.yaml except cobot.computed_torque.kp and kd, which are
critically damped at the profile's w: kp = w^2 and kd = 2 w on every joint. w = 50 (premium_fr3),
40 (mid_kr1205), 20 rad/s (budget_xarm7); default.yaml keeps w = 80 (ideal hardware). Paths in
the files are relative to their directory, so they live next to default.yaml.
"""

import copy
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
OMEGA = {"premium_fr3": 50.0, "mid_kr1205": 40.0, "budget_xarm7": 20.0}


def load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("name", list(OMEGA))
def test_profile_controller_equals_default_except_gains(name):
    default = load(REPO / "config/default.yaml")
    profile = load(REPO / "config" / f"{name}.yaml")
    w = OMEGA[name]
    ct = profile["cobot"]["computed_torque"]
    assert ct["kp"] == [w * w] * 7, name
    assert ct["kd"] == [2.0 * w] * 7, name
    a, b = copy.deepcopy(default), copy.deepcopy(profile)
    for d in (a, b):
        del d["cobot"]["computed_torque"]["kp"]
        del d["cobot"]["computed_torque"]["kd"]
    assert a == b, f"config/{name}.yaml differs from default.yaml beyond the gains"


def test_default_keeps_omega_80():
    ct = load(REPO / "config/default.yaml")["cobot"]["computed_torque"]
    assert ct["kp"] == [6400.0] * 7 and ct["kd"] == [160.0] * 7
