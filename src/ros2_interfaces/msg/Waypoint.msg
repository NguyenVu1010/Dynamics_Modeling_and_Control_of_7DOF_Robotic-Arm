# One waypoint of a FollowWaypoints goal or PreviewWaypoints request (design
# docs/design/2026-10-29-waypoint-gui-design.md 6.4, file format in docs/conventions.md).
geometry_msgs/Pose pose  # TCP pose in base_link (the controller applies cobot.tcp_offset)
float64 duration_s       # time from the previous point, s; 0 = from the Cartesian limits
bool stop                # come to rest here (the last waypoint always stops)
