# model: URDF -> Model<N>. parse_urdf_chain reads the URDF with tinyxml2.
find_package(tinyxml2 REQUIRED)
cobot_add_module(model
  SOURCES src/model.cpp src/urdf_parser.cpp
  DEPS cobot::common tinyxml2::tinyxml2)
