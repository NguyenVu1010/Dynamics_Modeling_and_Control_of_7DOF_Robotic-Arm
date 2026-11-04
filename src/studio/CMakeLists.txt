# cobot_studio (ADR-021): the core library (no Qt) and, with COBOT_BUILD_STUDIO, the Qt app.
add_subdirectory(core)
if(COBOT_BUILD_STUDIO AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/CMakeLists.txt")
  add_subdirectory(app)
endif()
