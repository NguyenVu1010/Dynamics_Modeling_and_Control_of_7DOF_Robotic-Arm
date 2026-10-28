# Shared compile settings. CI and tools/wsl/build.sh pass -DCOBOT_WERROR=ON.
option(COBOT_WERROR "Treat warnings as errors" OFF)
option(COBOT_COVERAGE "Instrument core and tests for gcov" OFF)
set(COBOT_ARCH_FLAGS "" CACHE STRING "Extra architecture flags, e.g. -mcpu=cortex-a76 for Pi5")

function(cobot_set_target_defaults target)
  get_target_property(type ${target} TYPE)
  if(type STREQUAL "INTERFACE_LIBRARY")
    target_compile_features(${target} INTERFACE cxx_std_17)
    return()
  endif()
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  if(COBOT_WERROR)
    target_compile_options(${target} PRIVATE -Werror)
  endif()
  if(COBOT_ARCH_FLAGS)
    separate_arguments(arch_flags UNIX_COMMAND "${COBOT_ARCH_FLAGS}")
    target_compile_options(${target} PRIVATE ${arch_flags})
  endif()
  if(COBOT_COVERAGE)
    target_compile_options(${target} PRIVATE --coverage -O0)
    target_link_options(${target} PRIVATE --coverage)
  endif()
endfunction()

# Declare a core module. Without SOURCES it is an INTERFACE (header-only) target; a feature
# that adds the first .cpp passes SOURCES and the target becomes STATIC. DEPS are other
# cobot::<module> targets or imported targets; the linker then enforces the layering.
function(cobot_add_module name)
  cmake_parse_arguments(ARG "" "" "SOURCES;DEPS" ${ARGN})
  set(target cobot_${name})
  if(ARG_SOURCES)
    add_library(${target} STATIC ${ARG_SOURCES})
    target_include_directories(${target} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_link_libraries(${target} PUBLIC ${ARG_DEPS})
    set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  else()
    add_library(${target} INTERFACE)
    target_include_directories(${target} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_link_libraries(${target} INTERFACE ${ARG_DEPS})
  endif()
  add_library(cobot::${name} ALIAS ${target})
  set_target_properties(${target} PROPERTIES EXPORT_NAME ${name})
  cobot_set_target_defaults(${target})
endfunction()
