# A drive command for mowbite_safety (S2), which alone writes hw/cmd_vel; the logic tags each with its source.
# The logic stops the mower: passes always and ends the drive of the source that moves.
uint8 SOURCE_STOP=0
# Remote control, only while an area is recorded.
uint8 SOURCE_TELEOP=1
# The follower: mowing, transit, drive_to, undocking and docking.
uint8 SOURCE_FOLLOWER=2
# The calibration drive (mowbite_calibration), forward only.
uint8 SOURCE_CALIBRATION=3
uint8 source
geometry_msgs/Twist twist
