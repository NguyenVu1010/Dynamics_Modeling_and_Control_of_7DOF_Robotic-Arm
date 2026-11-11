# The noise level selector of the Configuration tab (feature studio-noise-level, studio config UI
# design 3.4 and 4, ADR-021): the `noiseLevel` card on the Sensor / noise page driven through its
# object names in an offscreen QApplication, against the real budget_xarm7 and ideal pairs. Built
# only with -DCOBOT_BUILD_STUDIO=ON (src/studio/app). The QApplication main and the wait helpers
# are tests/component/studio_app's; one ctest entry with the offscreen platform.
if(NOT TARGET cobot_studio_ui)
  return()
endif()
find_package(Qt5 5.15 REQUIRED COMPONENTS Widgets Test Charts)
find_package(yaml-cpp REQUIRED)

add_executable(test_studio_noise_selector ../studio_app/main.cpp test_studio_noise_selector.cpp)
cobot_set_target_defaults(test_studio_noise_selector)
target_include_directories(test_studio_noise_selector PRIVATE
  ${PROJECT_SOURCE_DIR}/src/studio/app
  ${CMAKE_CURRENT_SOURCE_DIR}/../studio_app)
target_link_libraries(test_studio_noise_selector PRIVATE cobot_studio_ui cobot_studio_core
  Qt5::Widgets Qt5::Test Qt5::Charts yaml-cpp::yaml-cpp GTest::gtest)
target_compile_definitions(test_studio_noise_selector PRIVATE
  COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
  COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output")
target_compile_options(test_studio_noise_selector PRIVATE -UNDEBUG)
add_test(NAME test_studio_noise_selector COMMAND test_studio_noise_selector)
set_tests_properties(test_studio_noise_selector PROPERTIES ENVIRONMENT QT_QPA_PLATFORM=offscreen
  TIMEOUT 300)
