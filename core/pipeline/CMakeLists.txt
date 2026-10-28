# pipeline: ControlLoop, config loading (only place that parses YAML), CSV log.
cobot_add_module(pipeline DEPS cobot::control cobot::planning cobot::imperfect cobot::hal)
