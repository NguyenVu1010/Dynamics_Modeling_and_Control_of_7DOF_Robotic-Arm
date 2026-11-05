# The catalog keys the Gazebo path cannot honour (feature studio-control, design 6.5, the
# controller's on_configure). Skipped where src/studio is not built.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_catalog_gazebo test_studio_catalog_gazebo.cpp
  LIBS cobot_studio_core cobot::pipeline)
