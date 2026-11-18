# hw/obstacles: what one obstacle sensor of the platform sees on the ground, at a fixed rate of at least 4 Hz, also
# when it finds nothing. header.stamp is when the frame was taken, by the host clock; header.frame_id is base_link.
# A sensor that has no message for a while counts as without sight.
std_msgs/Header header
# One of obstacle_sources of hw/capabilities, e.g. the camera id "stereo".
string source

# Whether the sensor sees now. Without sight obstacles is empty and tells nothing about the ground.
bool sight
# The names of hw/cameras where they mean the same: NOT_SET_UP also for a missing model, calibration or mount,
# NO_SIGNAL also for frames that stand still.
string REASON_NOT_SET_UP=NOT_SET_UP
string REASON_TOO_DARK=TOO_DARK
string REASON_TOO_BRIGHT=TOO_BRIGHT
string REASON_COVERED=COVERED
string REASON_NO_SIGNAL=NO_SIGNAL
string REASON_TOO_HOT=TOO_HOT
string REASON_SELF_TEST=SELF_TEST
string REASON_LOCKED_AFTER_CRASH=LOCKED_AFTER_CRASH
# Empty with sight.
string reason
# Why, in a few words for the log ("SoC 81 C").
string detail

# Where the sensor sees the ground, as the platform computes it from its mounting and lens: a sector around the point
# view_x, view_y in base_link, from view_angle_right to view_angle_left (rad, counter-clockwise from x of base_link)
# and from view_range_min to view_range_max away from that point (m). Only what lies inside counts as seen: a place
# outside it is not seen free, whatever obstacles holds.
float32 view_x
float32 view_y
float32 view_angle_left
float32 view_angle_right
float32 view_range_min
float32 view_range_max

Obstacle[] obstacles
