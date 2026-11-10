# The noise level of the Configuration tab (feature studio-noise-level, studio config UI design 3.4,
# ADR-021): noise_factor, noise_level and apply_noise_level on a hand-made base map and on the real
# budget_xarm7 and ideal pairs. Qt-free: src/studio/core/include/cobot/studio/noise_level.hpp in
# cobot_studio_core. Skipped where src/studio is not built (the Pi 5 image).
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_noise_level test_noise_level.cpp LIBS cobot_studio_core cobot::pipeline)
