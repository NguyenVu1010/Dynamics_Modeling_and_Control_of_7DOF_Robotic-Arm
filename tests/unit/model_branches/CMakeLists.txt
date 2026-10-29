# Review gaps of feature model-urdf: fixed links before the first revolute joint, moving side
# branches, file name in parse errors. Own directory so the frozen model CMakeLists stays as is.
cobot_add_test(test_model_branches test_model_branches.cpp LIBS cobot::model)
target_include_directories(test_model_branches PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../model)
