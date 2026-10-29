"""tools/run_e2e.py gazebo branch passes the scenario's config and imperfection profile to the
launch as absolute `config_path:=` and `imperfections_path:=` arguments (unit level: Popen and
the wait are replaced, no ros2 and no Gazebo)."""

from pathlib import Path

import pytest

import run_e2e

REPO = Path(__file__).resolve().parents[2]


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
    return dict(a.split(":=", 1) for a in cmd[4:])


@pytest.mark.parametrize(
    "imperfections", ["config/imperfections/ideal.yaml", "config/imperfections/realistic.yaml"]
)
def test_gazebo_launch_gets_config_and_imperfection_paths(tmp_path, launched, imperfections):
    s = {"id": "T", "runner": "gazebo", "kind": "run", "config": "config/default.yaml"}
    s |= {"imperfections": imperfections, "seed": 1}
    ok, detail = run_e2e.run_gazebo(s, tmp_path, False)
    assert not ok and "launch exited with 3" in detail
    assert len(launched) == 1
    args = launch_args(launched[0])
    assert Path(args["config_path"]) == REPO / "config/default.yaml"
    assert Path(args["imperfections_path"]) == REPO / imperfections
    assert Path(args["config_path"]).is_absolute() and Path(args["config_path"]).is_file()
    assert Path(args["imperfections_path"]).is_file()
    assert args["log_path"] == str(tmp_path / "run.csv")


def test_absolute_scenario_paths_are_passed_unchanged(tmp_path, launched):
    config = (REPO / "config/default.yaml").resolve()
    imperfections = (REPO / "config/imperfections/pessimistic.yaml").resolve()
    s = {"id": "T", "runner": "gazebo", "kind": "run", "config": str(config)}
    s |= {"imperfections": str(imperfections), "seed": 1}
    run_e2e.run_gazebo(s, tmp_path, False)
    args = launch_args(launched[0])
    assert args["config_path"] == str(config)
    assert args["imperfections_path"] == str(imperfections)
