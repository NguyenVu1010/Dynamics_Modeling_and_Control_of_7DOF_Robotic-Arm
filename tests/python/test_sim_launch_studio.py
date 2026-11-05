"""sim/launch/sim.launch.py, studio bridge arguments (feature controller-bridge).

- `log_mode` (default "") reaches the cobot_controller as the string ROS parameter `log_mode`,
  like `command_source`; empty: no parameter (the config file's cobot.log.mode applies);
- `studio` (default "false"): "true" sets `command_source: external` and
  `log_mode: recorder_live` unless the argument is given explicitly, which wins.
Same fake launch API as test_sim_launch_realism.py (no ROS, no Gazebo).
"""

import test_sim_launch_realism as realism

run_launch = realism.run_launch  # the fixture: fake launch modules, then generate and perform


def test_arguments_are_declared_with_their_defaults(run_launch):
    declared = run_launch().declared
    assert declared.get("log_mode") == ""
    assert declared.get("studio") == "false"


def test_default_launch_sets_neither_parameter(run_launch):
    params = run_launch().controller_params()
    assert "log_mode" not in params
    assert "command_source" not in params


def test_log_mode_reaches_the_controller(run_launch):
    params = run_launch(log_mode="recorder_live").controller_params()
    assert params["log_mode"] == "recorder_live"
    assert "command_source" not in params


def test_studio_sets_external_and_recorder_live(run_launch):
    params = run_launch(studio="true").controller_params()
    assert params["command_source"] == "external"
    assert params["log_mode"] == "recorder_live"


def test_explicit_log_mode_wins_over_studio(run_launch):
    params = run_launch(studio="true", log_mode="recorder").controller_params()
    assert params["log_mode"] == "recorder"
    assert params["command_source"] == "external"


def test_explicit_command_source_wins_over_studio(run_launch):
    params = run_launch(studio="true", command_source="config").controller_params()
    assert params["command_source"] == "config"
    assert params["log_mode"] == "recorder_live"


def test_studio_false_changes_nothing(run_launch):
    params = run_launch(studio="false").controller_params()
    assert "log_mode" not in params
    assert "command_source" not in params


def test_studio_keeps_the_other_parameters(run_launch, tmp_path):
    csv = str(tmp_path / "run.csv")
    params = run_launch(studio="true", log_path=csv, seed="7").controller_params()
    assert params["log_path"] == csv
    assert params["seed"] == 7
