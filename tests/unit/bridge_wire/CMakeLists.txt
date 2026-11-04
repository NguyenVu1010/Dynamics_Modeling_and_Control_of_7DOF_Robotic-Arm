# Wire types, to_sample, copy_text and the seqlock slot of the studio bridge (feature shm-bridge,
# cobot_studio design 6.5, ADR-022). Headers: src/core/bridge/include/cobot/bridge/*.hpp. The
# allocation test replaces the global operator new, so it is a binary of its own.
set(bridge_libs cobot::pipeline)
if(TARGET cobot::bridge)
  list(APPEND bridge_libs cobot::bridge)
endif()
cobot_add_test(test_bridge_wire test_wire.cpp LIBS ${bridge_libs})
cobot_add_test(test_bridge_wire_no_alloc test_wire_no_alloc.cpp LIBS ${bridge_libs})
target_include_directories(test_bridge_wire_no_alloc
                           PRIVATE ${PROJECT_SOURCE_DIR}/tests/unit/bridge_endpoint)
