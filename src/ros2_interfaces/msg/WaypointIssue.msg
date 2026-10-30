# One path validation finding (pipeline::WaypointIssue, design 6.2).
uint32 waypoint  # index of the waypoint whose segment or node shows the issue
string issue     # unreachable | joint_limit | joint_velocity | torque
int32 joint      # 0-based joint for joint_* and torque, -1 otherwise
float64 value    # worst value seen (m, rad, rad/s or tau ratio, see path_validation.hpp)
float64 limit    # the limit it is compared with
