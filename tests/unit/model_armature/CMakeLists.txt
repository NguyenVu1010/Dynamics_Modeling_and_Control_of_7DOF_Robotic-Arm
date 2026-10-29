# Feature model-armature: <cobot_armature value> per revolute joint (reflected rotor inertia
# J_in * N^2). Own directory so the frozen model CMakeLists stays as is.
cobot_add_test(test_model_armature test_armature_parser.cpp LIBS cobot::model)
target_include_directories(test_model_armature PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../model)
