# Feature studio-3d (cobot_studio design 6.6 and 7, ADR-021): RobotScene (URDF visuals placed by
# forward_kinematics), the STL loader and the playback helpers of cobot_studio_core (no Qt).
# The Pi 5 image has no src/studio, so the directory is skipped when the library is absent.
if(NOT TARGET cobot_studio_core)
  return()
endif()

cobot_add_test(test_studio_scene test_robot_scene.cpp test_stl.cpp test_playback.cpp
  LIBS cobot_studio_core cobot_bench_run cobot::pipeline)
# visual_poses runs at 30 Hz per drawn run: the replaced global operator new counts allocations
# inside the compiled library, so this check is a binary of its own.
cobot_add_test(test_studio_scene_no_alloc test_robot_scene_no_alloc.cpp
  LIBS cobot_studio_core cobot::pipeline)
