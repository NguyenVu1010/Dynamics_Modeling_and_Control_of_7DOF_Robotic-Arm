# Follow-up of feature hardware-catalog (ADR-023): cases the review of #76 listed as uncovered. A
# directory of its own, so the tests frozen by #76 stay untouched. Helpers live with those tests.
cobot_add_test(test_pipeline_catalog_errors test_catalog_errors.cpp)
target_include_directories(test_pipeline_catalog_errors PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_hardware_model
  ${CMAKE_CURRENT_SOURCE_DIR}/../pipeline_task)
