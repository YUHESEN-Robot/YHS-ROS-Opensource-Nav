# timoo LIDAR scan packets.

Header           header         # standard ROS message header
bool gps_status
Header gps_timestamp
float32[16] vertical_angle_list
