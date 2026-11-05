# The Control tab of cobot_studio (feature studio-control, cobot_studio design 5.3, 6.7, 6.8, 7;
# ADR-021, ADR-022): the window driven through its object names against bench::serve on a thread
# of the test (config/default.yaml + ideal, region /cobot_cobot_test_control_<pid>). Built only
# with -DCOBOT_BUILD_STUDIO=ON. Own main (QApplication), so one CTest test, run serially, 300 s.
# The liveRobotView test needs an OpenGL context (studio-3d's rule): under xvfb-run (xcb) when it
# is installed, else offscreen, where that test skips when QOpenGLContext::create() fails.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Gui Test Charts 3DCore 3DRender 3DExtras)

add_executable(test_studio_control test_studio_control.cpp)
cobot_set_target_defaults(test_studio_control)
target_include_directories(test_studio_control PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app ${PROJECT_SOURCE_DIR}/src/bench
  ${PROJECT_SOURCE_DIR}/tests/component/studio_app)
target_link_libraries(test_studio_control PRIVATE cobot_studio_ui cobot_studio_core cobot_bench_run
  cobot::bridge Qt5::Widgets Qt5::Gui Qt5::Test Qt5::Charts Qt5::3DCore Qt5::3DRender
  Qt5::3DExtras GTest::gtest)
target_compile_definitions(test_studio_control PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_control PRIVATE -UNDEBUG)

find_program(XVFB_RUN xvfb-run)
if(XVFB_RUN)
  add_test(NAME test_studio_control COMMAND ${XVFB_RUN} -a $<TARGET_FILE:test_studio_control>)
  set_tests_properties(test_studio_control PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=xcb TIMEOUT 300
                       RUN_SERIAL TRUE)
else()
  add_test(NAME test_studio_control COMMAND test_studio_control)
  set_tests_properties(test_studio_control PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
                       TIMEOUT 300 RUN_SERIAL TRUE)
endif()
