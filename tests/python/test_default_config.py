import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / "config/default.yaml"
IDEAL = REPO / "config/imperfections/ideal.yaml"


@pytest.fixture(scope="module")
def cfg():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["cobot"]


@pytest.fixture(scope="module")
def urdf_limits(cfg):
    root = ET.parse((CONFIG.parent / cfg["urdf_path"]).resolve()).getroot()
    limits = []
    for joint in root.findall("joint"):
        if joint.get("type") == "revolute":
            lim = joint.find("limit")
            limits.append((float(lim.get("lower")), float(lim.get("upper"))))
    return limits


def test_home_within_limits_with_margin(cfg, urdf_limits):
    margin = math.radians(5.0)
    assert len(urdf_limits) == 7
    for k, (deg, (lo, hi)) in enumerate(zip(cfg["home_q_deg"], urdf_limits, strict=True)):
        q = math.radians(deg)
        assert lo + margin <= q <= hi - margin, f"joint_{k + 1}: {deg} deg outside limits"


def test_vector_lengths(cfg):
    assert len(cfg["home_q_deg"]) == 7
    assert len(cfg["computed_torque"]["kp"]) == 7
    assert len(cfg["computed_torque"]["kd"]) == 7
    assert len(cfg["clik"]["kp"]) == 6
    assert len(cfg["clik"]["kd"]) == 6
    assert len(cfg["gravity_b"]) == 3
    assert len(cfg["trajectory"]["goal_position_m"]) == 3
    assert len(cfg["trajectory"]["goal_rpy_deg"]) == 3


def test_rates_and_gains_positive(cfg):
    assert cfg["control_rate_hz"] > 0
    assert cfg["trajectory"]["duration_s"] > 0
    assert cfg["trajectory"]["hold_s"] >= 0
    for group in ("computed_torque", "clik"):
        assert all(v > 0 for v in cfg[group]["kp"])
        assert all(v > 0 for v in cfg[group]["kd"])


def test_urdf_path_exists(cfg):
    assert (CONFIG.parent / cfg["urdf_path"]).resolve().is_file()


def test_ideal_profile_is_all_zero():
    imp = yaml.safe_load(IDEAL.read_text(encoding="utf-8"))["imperfections"]
    keep_nonzero = {"qd_filter_hz", "ripple_periods_per_rev"}
    for group in ("sensor", "actuator", "friction", "model", "timing"):
        for key, value in imp[group].items():
            if key in keep_nonzero or key == "source" or isinstance(value, bool):
                continue
            assert value == 0, f"{group}.{key} = {value} in the ideal profile"


def test_friction_compensation_is_80_percent_of_realistic_plant_friction(cfg):
    # Nominal identified values: deliberately 80 % of the realistic profile's plant friction
    # (identification is never exact; under-compensation is the safe side).
    comp = cfg["friction_compensation"]
    assert comp["enabled"] is False  # off until SimPlant applies Coulomb friction stably
    assert comp["velocity_eps_rad_s"] == 0.01
    real = REPO / "config/imperfections/realistic.yaml"
    plant = yaml.safe_load(real.read_text(encoding="utf-8"))["imperfections"]["friction"]
    for key in ("coulomb_nm", "viscous_nm_s_rad"):
        assert len(comp[key]) == 7
        assert all(v >= 0 for v in comp[key])
        for k, (c, p) in enumerate(zip(comp[key], plant[key], strict=True)):
            assert c == pytest.approx(0.8 * p, rel=1e-12), f"{key}[{k}]"
