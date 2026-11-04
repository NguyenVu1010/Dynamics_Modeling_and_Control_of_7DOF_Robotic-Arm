# Server and client endpoints of the studio bridge in one process (feature shm-bridge, cobot_studio
# design 6.5, ADR-022): region life cycle, layout checks, authority, the three rings, the seqlock
# under load, owner state and session. Regions are named /cobot_test_<pid>_<n> and removed by RAII
# guards. The allocation test replaces the global operator new, so it is a binary of its own.
set(bridge_libs cobot::pipeline)
if(TARGET cobot::bridge)
  list(APPEND bridge_libs cobot::bridge)
endif()
find_package(Threads REQUIRED)
cobot_add_test(test_bridge_endpoint test_endpoint.cpp test_endpoint_load.cpp
  LIBS ${bridge_libs} Threads::Threads)
cobot_add_test(test_bridge_endpoint_no_alloc test_endpoint_no_alloc.cpp LIBS ${bridge_libs})
