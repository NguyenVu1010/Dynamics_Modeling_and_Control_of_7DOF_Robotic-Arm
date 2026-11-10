# Presentation fields of the parameter catalog (feature studio-theme-catalog, studio config UI
# design 3.2): labels, descriptions, display units and format_display. Skipped where src/studio
# is not built.
if(NOT TARGET cobot_studio_core)
  return()
endif()
cobot_add_test(test_studio_catalog_display test_studio_catalog_display.cpp
  LIBS cobot_studio_core cobot::pipeline)
