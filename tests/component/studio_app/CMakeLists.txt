# cobot_studio smoke test (feature studio-app, cobot_studio design 7, ADR-021): the widgets of
# cobot_studio_ui driven through their object names in an offscreen QApplication. Built only with
# -DCOBOT_BUILD_STUDIO=ON (src/studio/app). Own main (QApplication), so not cobot_add_test: the
# binary is one ctest entry with the offscreen platform and a 300 s timeout.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts)

add_executable(test_studio_app main.cpp test_studio_app.cpp)
cobot_set_target_defaults(test_studio_app)
target_include_directories(test_studio_app PRIVATE ${PROJECT_SOURCE_DIR}/src/studio/app)
target_link_libraries(test_studio_app PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Charts GTest::gtest)
target_compile_definitions(test_studio_app PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_app PRIVATE -UNDEBUG)
add_test(NAME test_studio_app COMMAND test_studio_app)
set_tests_properties(test_studio_app PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen TIMEOUT 300)
