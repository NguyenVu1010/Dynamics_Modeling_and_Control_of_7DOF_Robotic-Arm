# common: fixed-size types, Pose, Timestamp, Status/Result, logger, SO(3) helpers.
cobot_add_module(common SOURCES src/logger.cpp src/lie.cpp DEPS Eigen3::Eigen)
