# Host glue of the studio bridge (feature controller-bridge, cobot_studio design 6.5, ADR-022):
# bridge::LoopPort on the control thread and bridge::CommandService on the bridge thread, driven in
# one process by a deterministic harness (SimPlant, 1 ms fake clock, poll every 5 steps) with a
# ClientEndpoint of the same process. Linked by the real target name, so a missing module is a
# link error, not a configure error.
cobot_add_test(test_bridge_service test_bridge_service.cpp LIBS cobot_bridge cobot::pipeline)

# Replaces the global operator new: its own binary.
cobot_add_test(test_bridge_service_no_alloc test_bridge_service_no_alloc.cpp
  LIBS cobot_bridge cobot::pipeline)
