# State of mowbite_safety (S2) on safety/status at 10 Hz.
builtin_interfaces/Time stamp
# TILT and FENCE (bits as in hw/status) that the filter keeps until a release passes it.
uint32 latched
# Angle of the body to the vertical in degrees, NaN without a recent IMU sample.
float32 tilt_deg
# m base_link lies outside of the allowed areas (0 inside), NaN without a map that fits or a recent pose.
float32 fence_distance
# Autonomous drives may pass: a safety measurement, a recent pose of the accuracy asked for with a heading, a map that
# fits the datum.
bool autonomous_ok
# Drives in the station frame (SOURCE_DOCK) may pass: a safety measurement and a recent pose from the dock marker
# (localization/dock_pose) of dock_max_position_accuracy with a heading.
bool dock_ok
# The source that moves the mower now, as in DriveCommand; SOURCE_STOP while none does.
uint8 owner
# Moving commands refused since the start, and why the last one was.
uint32 refused
string last_refusal
