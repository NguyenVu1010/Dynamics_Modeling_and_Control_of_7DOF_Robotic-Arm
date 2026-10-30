# core: C++17 library, no ROS. Layering (enforced by target links):
#   pipeline -> {control, planning, imperfect} -> {dynamics, kinematics} -> model -> common
#   control -> hal, imperfect -> hal, hal -> common
cmake_minimum_required(VERSION 3.22)
project(cobot_core VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
include(GNUInstallDirs)
include(cobot_warnings)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(COBOT_MODULES common model kinematics dynamics hal planning control imperfect pipeline)
foreach(module IN LISTS COBOT_MODULES)
  add_subdirectory(${module})
endforeach()

set(COBOT_TARGETS)
foreach(module IN LISTS COBOT_MODULES)
  list(APPEND COBOT_TARGETS cobot_${module})
  install(DIRECTORY ${module}/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endforeach()
install(TARGETS ${COBOT_TARGETS} EXPORT cobot_coreTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(EXPORT cobot_coreTargets NAMESPACE cobot:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cobot_core)
include(CMakePackageConfigHelpers)
configure_package_config_file(../../cmake/cobot_coreConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/cobot_coreConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cobot_core)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cobot_coreConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cobot_core)
