uint8 RATE = 0
uint8 NORMAL = 1

time stamp
uint8 mode

#RATE
float64 rate_yaw
float64 rate_pitch

#NORMAL
float64 yaw_target_pos
float64 pitch_target_pos

#DIRECT
geometry_msgs/PointStamped target_pos
