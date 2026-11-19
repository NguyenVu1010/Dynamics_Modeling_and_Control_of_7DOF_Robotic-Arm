# Screenshots of the Configuration tab (feature studio-shots, design
# docs/design/2026-11-10-studio-config-ui-design.md 5 item 4): save_screenshots writes one PNG per
# page plus the noise x2 view, themed, in an offscreen QApplication, and leaves the configuration as
# it found it. Built only with -DCOBOT_BUILD_STUDIO=ON. The QApplication main is
# tests/component/studio_app's.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts)

add_executable(test_studio_shots ../studio_app/main.cpp test_studio_shots.cpp)
cobot_set_target_defaults(test_studio_shots)
target_include_directories(test_studio_shots PRIVATE ${PROJECT_SOURCE_DIR}/src/studio/app)
target_link_libraries(test_studio_shots PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Charts
  GTest::gtest)
target_compile_definitions(test_studio_shots PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_shots PRIVATE -UNDEBUG)
add_test(NAME test_studio_shots COMMAND test_studio_shots)
set_tests_properties(test_studio_shots PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 300)
