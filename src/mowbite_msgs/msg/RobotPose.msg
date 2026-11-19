# localization/pose: the estimated pose of base_link in the map frame.
std_msgs/Header header
float64 x
float64 y
# Counter-clockwise from x (east), rad, within -pi..pi.
float64 heading
bool heading_valid
# Expected errors, m and rad.
float32 position_accuracy
float32 heading_accuracy
# position_accuracy without the part of heading_accuracy over the lever of the antenna: for a check that sets the
# heading itself, as the reference check on the docking station does.
float32 position_accuracy_at_heading
# A GNSS solution went into the estimate recently.
bool gnss_recent
# s since the estimate took its last GNSS fix, 999 without one. 0 on localization/dock_pose.
float32 fix_age
# Along the heading, negative when backing up, m/s.
float64 speed
# Of x, y and heading, row by row.
float64[9] covariance
