# The studio theme (feature studio-theme-catalog, studio config UI design 3.5): the vendored
# Breeze light stylesheet plus studio.qss from the resources of cobot_studio_ui, applied to the
# QApplication, and a MainWindow built under it without stylesheet parse warnings. Built only
# with -DCOBOT_BUILD_STUDIO=ON (src/studio/app). The QApplication main and the wait helpers are
# tests/component/studio_app's; one ctest entry with the offscreen platform.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Charts)

add_executable(test_studio_theme ../studio_app/main.cpp test_studio_theme.cpp)
cobot_set_target_defaults(test_studio_theme)
target_include_directories(test_studio_theme PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../studio_app)
target_link_libraries(test_studio_theme PRIVATE cobot_studio_ui Qt5::Widgets Qt5::Charts
  GTest::gtest)
target_compile_definitions(test_studio_theme PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_theme PRIVATE -UNDEBUG)
add_test(NAME test_studio_theme COMMAND test_studio_theme)
set_tests_properties(test_studio_theme PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 120)
