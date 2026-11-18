# One obstacle the OS keeps track of, in the map frame.
uint32 id
# As Obstacle.msg.
string kind
string class_name
float32 score
float64 x
float64 y
float32 radius
# 1 sigma of the position along its larger axis (m).
float32 sigma
# Found in enough frames to count.
bool confirmed
# Its place lies in the view of a sensor with sight now.
bool in_view
# s since a sensor last found it.
float32 age
# Distance of its edge from the nose of the mower along the heading (m, negative behind the nose) and from the middle
# line (m, left positive), at the pose of this status.
float32 ahead
float32 aside
