"""sim/launch/sim.launch.py passes a `log_stream_ms` launch argument (feature blackbox-log).

Default "0" (no streaming: the controller writes its log on ~/dump_log, deactivate and cleanup);
a value reaches the cobot_controller as the int ROS parameter `log_stream_ms`, like `log_path`.
Same fake launch API as test_sim_launch_realism.py (no ROS, no Gazebo).
"""

import test_sim_launch_realism as realism

run_launch = realism.run_launch  # the fixture: fake launch modules, then generate and perform


def test_log_stream_ms_is_declared_with_default_zero(run_launch):
    assert run_launch().declared.get("log_stream_ms") == "0"


def test_default_does_not_stream(run_launch):
    params = run_launch().controller_params()
    assert params.get("log_stream_ms", 0) == 0


def test_log_stream_ms_reaches_the_controller_as_int(run_launch):
    value = run_launch(log_stream_ms="200").controller_params()["log_stream_ms"]
    assert type(value) is int and value == 200


def test_log_path_still_reaches_the_controller(run_launch, tmp_path):
    csv = str(tmp_path / "run.csv")
    params = run_launch(log_path=csv, log_stream_ms="200").controller_params()
    assert params["log_path"] == csv
