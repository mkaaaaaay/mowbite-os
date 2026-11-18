# logic/drive_gate at 10 Hz: whether a path of the follower with gate set may move now. Older than gate_timeout of the
# follower counts as no: the logic hangs, and so does its hold for obstacles.
std_msgs/Header header
# False while the logic holds the mower for an obstacle.
bool go
# A person or animal ahead: at most speed_slow of the follower.
bool slow
