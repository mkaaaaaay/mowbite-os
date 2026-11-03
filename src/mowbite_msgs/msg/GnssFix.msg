# gps/absolute_pose: one solution of the GNSS receiver for its antenna, in the map frame (x east, y north, z up).
# header.stamp is when the driver read it.
std_msgs/Header header
# UTC time of day of the solution by the receiver's clock, ms.
uint32 receiver_time_ms

uint8 NONE=0
uint8 SINGLE=1
uint8 DGPS=2
uint8 RTK_FLOAT=3
# Only with fresh corrections from the expected source; otherwise the driver reports RTK_FLOAT.
uint8 RTK_FIXED=4
uint8 DEAD_RECKONING=5
uint8 solution

float64 x
float64 y
float64 z
# Expected horizontal error, m.
float32 accuracy

# Velocity of the antenna over ground, m/s; only meaningful while velocity_valid.
bool velocity_valid
float64 velocity_east
float64 velocity_north
