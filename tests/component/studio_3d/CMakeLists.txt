# Feature studio-3d (cobot_studio design 6.9 and 7, ADR-021): the Qt3D playback view of the
# Tracking tab, driven through the widgets' object names. Needs an OpenGL context: under
# xvfb-run (xcb) when it is installed, else offscreen, where every test skips cleanly when
# QOpenGLContext::create() fails. Own main (QApplication), so no gtest discovery: one CTest test.
if(NOT TARGET cobot_studio_ui)
  return()
endif()

find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Gui 3DCore 3DRender 3DExtras)

add_executable(test_studio_3d test_studio_3d.cpp)
cobot_set_target_defaults(test_studio_3d)
target_link_libraries(test_studio_3d PRIVATE cobot_studio_ui cobot_studio_core cobot_bench_run
                      Qt5::Widgets Qt5::Gui Qt5::3DCore Qt5::3DRender Qt5::3DExtras GTest::gtest)
target_compile_definitions(test_studio_3d PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_3d PRIVATE -UNDEBUG)

find_program(XVFB_RUN xvfb-run)
if(XVFB_RUN)
  add_test(NAME test_studio_3d COMMAND ${XVFB_RUN} -a $<TARGET_FILE:test_studio_3d>)
  set_tests_properties(test_studio_3d PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=xcb TIMEOUT 300)
else()
  add_test(NAME test_studio_3d COMMAND test_studio_3d)
  set_tests_properties(test_studio_3d PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
                       TIMEOUT 300)
endif()
