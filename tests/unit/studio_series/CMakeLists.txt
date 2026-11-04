# decimate_minmax of cobot_studio_core (feature studio-app, cobot_studio design 5.3, ADR-021): the
# min/max decimation every chart series goes through. No Qt, so it is built wherever
# cobot_studio_core is (the Pi 5 image has no src/studio).
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_series test_decimate_minmax.cpp LIBS cobot_studio_core)
