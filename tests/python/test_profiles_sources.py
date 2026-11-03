"""Traceability of the three reference profiles (feature profiles, realism-2 design G3).

Every group of the schema is present in config/imperfections/{premium_fr3, mid_kr1205,
budget_xarm7}.yaml (sensor, actuator, friction, model, timing, comm, transmission), and every
group (every mapping directly under `imperfections`) has a non-empty `source` string that says
for its numbers whether they are published, derived or assumed (design section 7, "Co so" column;
anti-goal: an assumed value presented as a published one).
"""

from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
PROFILES = ["premium_fr3", "mid_kr1205", "budget_xarm7"]
GROUPS = ["sensor", "actuator", "friction", "model", "timing", "comm", "transmission"]
KINDS = ("published", "derived", "assumed")


def groups_of(name: str) -> dict:
    path = REPO / "config/imperfections" / f"{name}.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return {k: v for k, v in data["imperfections"].items() if isinstance(v, dict)}


@pytest.mark.parametrize("name", PROFILES)
def test_every_schema_group_present(name):
    assert sorted(groups_of(name)) == sorted(GROUPS)


@pytest.mark.parametrize("name", PROFILES)
def test_every_group_names_its_source_kind(name):
    for group, body in groups_of(name).items():
        source = body.get("source")
        assert isinstance(source, str) and source.strip(), f"{name}: {group}.source empty"
        assert any(kind in source for kind in KINDS), f"{name}: {group}.source = {source!r}"
