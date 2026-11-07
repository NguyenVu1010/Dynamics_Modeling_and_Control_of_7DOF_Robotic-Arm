"""The tracking-law block of the controller configs (ADR-023, feature controller-registry).

config/default.yaml and the three profile controller configs carry the same explicit
`cobot.control: {algorithm: computed_torque}` block, and none of them has a `pd_gravity` section,
so test_profile_controllers.py still sees the four files differ only in the computed-torque gains.
"""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIGS = ["default", "premium_fr3", "mid_kr1205", "budget_xarm7"]


def load(name: str) -> dict:
    return yaml.safe_load((REPO / "config" / f"{name}.yaml").read_text(encoding="utf-8"))


@pytest.mark.parametrize("name", CONFIGS)
def test_control_block_selects_computed_torque(name):
    cobot = load(name)["cobot"]
    assert cobot.get("control") == {"algorithm": "computed_torque"}, name
    assert "pd_gravity" not in cobot, name
    keys = list(cobot)
    assert keys.index("control") == keys.index("computed_torque") + 1, f"{name}: after CT gains"


def test_control_block_identical_in_all_four():
    blocks = [load(name)["cobot"].get("control") for name in CONFIGS]
    assert all(b == blocks[0] for b in blocks), blocks
