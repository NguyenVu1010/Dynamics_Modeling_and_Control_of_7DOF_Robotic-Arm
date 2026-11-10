# The redesigned Configuration tab of cobot_studio (feature studio-config-layout, design
# docs/design/2026-11-10-studio-config-ui-design.md sections 2 to 4): the group list and pages,
# ParamCard and ParamSpin, shared and per-joint fields, undo, the change counter and the flows
# that must survive (no rounding writes, omega and kp/kd, Open, Save, Reset). Driven through the
# object names in an offscreen QApplication. Built only with -DCOBOT_BUILD_STUDIO=ON
# (src/studio/app). The QApplication main and the wait helpers are tests/component/studio_app's;
# one ctest entry with the offscreen platform and a 300 s timeout.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Test)

add_executable(test_studio_config_tab ../studio_app/main.cpp test_studio_config_tab.cpp)
cobot_set_target_defaults(test_studio_config_tab)
target_include_directories(test_studio_config_tab PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../studio_app)
target_link_libraries(test_studio_config_tab PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Test
  GTest::gtest)
target_compile_definitions(test_studio_config_tab PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_config_tab PRIVATE -UNDEBUG)
add_test(NAME test_studio_config_tab COMMAND test_studio_config_tab)
set_tests_properties(test_studio_config_tab PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 300)
