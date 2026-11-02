# cobot.limits in load_config and pipeline::controller_model (feature joint-limits). Own directory:
# the frozen pipeline_config CMakeLists stays as is.
cobot_add_test(test_config_limits test_config_limits.cpp)
