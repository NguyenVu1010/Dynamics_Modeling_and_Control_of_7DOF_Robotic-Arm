# Feature hardware-catalog (cobot_studio phase C PR 4, ADR-023): config/hardware/<model>.yaml ->
# pipeline::HardwareModel, its provenance rule (G10), active_imperfection_keys and config_for.
cobot_add_test(test_pipeline_hardware_model
  test_hardware_model.cpp
  test_provenance.cpp
  test_active_keys.cpp
  test_config_for.cpp)
