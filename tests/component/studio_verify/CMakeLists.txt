# The Verification tab of cobot_studio (feature studio-verify, cobot_studio design 10 G14/G16,
# ADR-023, ADR-021): the tab and its contract widgets, the read-only result area, and (release
# builds only) a quick job, a cancelled job and the comparison of two pairs, driven through the
# object names in an offscreen QApplication. Built only with -DCOBOT_BUILD_STUDIO=ON
# (src/studio/app). The QApplication main and the wait helpers are tests/component/studio_app's;
# one ctest entry with the offscreen platform and a 900 s timeout.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts)

add_executable(test_studio_verify ../studio_app/main.cpp test_studio_verify.cpp)
cobot_set_target_defaults(test_studio_verify)
target_include_directories(test_studio_verify PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app
  ${PROJECT_SOURCE_DIR}/src/bench
  ${CMAKE_CURRENT_SOURCE_DIR}/../studio_app)
target_link_libraries(test_studio_verify PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Charts
  GTest::gtest)
target_compile_definitions(test_studio_verify PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_verify PRIVATE -UNDEBUG)
add_test(NAME test_studio_verify COMMAND test_studio_verify)
set_tests_properties(test_studio_verify PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 900)
