"""E2E-09 (feature gazebo-realism): headless Gazebo with the realistic profile, seed 7, E2E-02
thresholds, and tools/run_e2e.py passing the profile and the seed to the launch (unit level:
Popen and the wait are replaced, no ros2 and no Gazebo)."""

from pathlib import Path

import pytest
import yaml

import run_e2e

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "tests/e2e/scenarios"
E2E_09 = SCENARIOS / "e2e_09_gazebo_realistic.yaml"
REALISTIC = REPO / "config/imperfections/realistic.yaml"
# E2E-02 thresholds: 5 mm, 2 deg, 0.02 rad.
G3_REAL = {"final_pos_err_m": 0.005, "final_rot_err_deg": 2.0, "rms_joint_err_rad": 0.02}


def load(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_e2e_09_scenario_follows_contract():
    s = load(E2E_09)
    assert s["id"] == "E2E-09"
    assert s["tags"] == ["gazebo"]
    assert s["runner"] == "gazebo" and s["kind"] == "run"
    assert (REPO / s["config"]).resolve() == (REPO / "config/default.yaml").resolve()
    assert (REPO / s["imperfections"]).resolve() == REALISTIC.resolve()
    assert type(s["seed"]) is int and s["seed"] == 7
    assert s["thresholds"] == G3_REAL


def test_e2e_09_thresholds_match_e2e_02():
    assert load(E2E_09)["thresholds"] == load(SCENARIOS / "e2e_02_realistic.yaml")["thresholds"]


def test_e2e_09_is_selected_by_the_gazebo_tag():
    scenarios = [load(p) for p in sorted(SCENARIOS.glob("*.yaml"))]
    assert [s["id"] for s in scenarios].count("E2E-09") == 1
    assert "E2E-09" in [s["id"] for s in scenarios if "gazebo" in s.get("tags", [])]


def test_realistic_profile_has_no_payload():
    # The Gazebo path rejects payload_kg != 0 (cannot be emulated by model perturbation).
    assert float(load(REALISTIC)["imperfections"]["model"]["payload_kg"]) == 0.0


class FakeProc:
    pid = 0

    def poll(self):
        return 3


@pytest.fixture
def launched(monkeypatch):
    calls = []

    def fake_popen(cmd, **kwargs):
        calls.append(cmd)
        return FakeProc()

    monkeypatch.setattr(run_e2e.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(run_e2e, "wait_run", lambda proc, csv, wait_s, run_ns: 3)
    monkeypatch.setattr(run_e2e, "stop_launch", lambda proc: None)
    return calls


def launch_args(cmd):
    assert cmd[:4] == ["ros2", "launch", "cobot_sim", "sim.launch.py"]
    pairs = [a.split(":=", 1) for a in cmd[4:]]
    names = [p[0] for p in pairs]
    assert len(names) == len(set(names)), cmd
    return dict(pairs)


def test_e2e_09_launch_gets_realistic_profile_and_seed(tmp_path, launched):
    run_e2e.run_gazebo(load(E2E_09), tmp_path, False)
    assert len(launched) == 1
    args = launch_args(launched[0])
    assert Path(args["imperfections_path"]) == REALISTIC
    assert Path(args["config_path"]) == REPO / "config/default.yaml"
    assert args["seed"] == "7"


@pytest.mark.parametrize("seed", [1, 7, 123])
def test_gazebo_launch_gets_scenario_seed(tmp_path, launched, seed):
    s = {"id": "T", "runner": "gazebo", "kind": "run", "config": "config/default.yaml"}
    s |= {"imperfections": "config/imperfections/realistic.yaml", "seed": seed}
    run_e2e.run_gazebo(s, tmp_path, False)
    assert launch_args(launched[0])["seed"] == str(seed)
