find_package(GTest REQUIRED)
include(GoogleTest)

# cobot_add_test(<name> <sources...> [LIBS <targets...>])
# Every test links cobot::pipeline (which brings every core module) unless LIBS is given.
# EIGEN_RUNTIME_NO_MALLOC + -UNDEBUG let tests assert that hot paths do not allocate
# (Eigen::internal::set_is_malloc_allowed(false)); test code only, never core/ or bench.
function(cobot_add_test name)
  cmake_parse_arguments(ARG "" "" "LIBS" ${ARGN})
  add_executable(${name} ${ARG_UNPARSED_ARGUMENTS})
  cobot_set_target_defaults(${name})
  if(NOT ARG_LIBS)
    set(ARG_LIBS cobot::pipeline)
  endif()
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest GTest::gtest_main)
  target_compile_definitions(${name} PRIVATE
    COBOT_REPO_DIR="${PROJECT_SOURCE_DIR}"
    COBOT_TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output"
    EIGEN_RUNTIME_NO_MALLOC)
  target_compile_options(${name} PRIVATE -UNDEBUG)
  gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

# Each feature adds its own tests/<level>/<module>/CMakeLists.txt; the glob picks it up, so
# parallel branches never edit a shared CMake file.
foreach(level unit component)
  file(GLOB subdirs LIST_DIRECTORIES true RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
       ${CMAKE_CURRENT_SOURCE_DIR}/${level}/*)
  foreach(dir IN LISTS subdirs)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt)
      add_subdirectory(${dir})
    endif()
  endforeach()
endforeach()
