# Host glue of the studio bridge (feature controller-bridge, cobot_studio design 6.5, ADR-022):
# bridge::LoopPort on the control thread and bridge::CommandService on the bridge thread, driven in
# one process by a deterministic harness (SimPlant, 1 ms fake clock, poll every 5 steps) with a
# ClientEndpoint of the same process. Linked by the real target name, so a missing module is a
# link error, not a configure error.
cobot_add_test(test_bridge_service test_bridge_service.cpp LIBS cobot_bridge cobot::pipeline)

# Replaces the global operator new: its own binary.
cobot_add_test(test_bridge_service_no_alloc test_bridge_service_no_alloc.cpp
  LIBS cobot_bridge cobot::pipeline)
# The replacement operator new/delete (allocation counter) makes GCC report
# -Wmismatched-new-delete once the replacement free() is inlined into the cleanup path of a
# new-expression (std::make_unique of a type whose constructor may throw): a false positive.
# Only this binary is exempt; every other target keeps the warning.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(test_bridge_service_no_alloc PRIVATE -Wno-mismatched-new-delete)
endif()
