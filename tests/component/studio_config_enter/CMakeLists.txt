# Review follow-up of feature studio-config-layout (design
# docs/design/2026-11-10-studio-config-ui-design.md section 2, "an unedited value is never
# rewritten"): pressing Enter in a field the user did not type in leaves a full-precision override
# untouched, for a per-joint field and the shared field. Offscreen QApplication, built only with
# -DCOBOT_BUILD_STUDIO=ON; the QApplication main and the wait helpers are tests/component/studio_app's.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Test)

add_executable(test_studio_config_enter ../studio_app/main.cpp test_studio_config_enter.cpp)
cobot_set_target_defaults(test_studio_config_enter)
target_include_directories(test_studio_config_enter PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../studio_app)
target_link_libraries(test_studio_config_enter PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Test
  GTest::gtest)
target_compile_definitions(test_studio_config_enter PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_config_enter PRIVATE -UNDEBUG)
add_test(NAME test_studio_config_enter COMMAND test_studio_config_enter)
set_tests_properties(test_studio_config_enter PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 300)
