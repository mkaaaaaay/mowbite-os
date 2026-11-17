# A drive command for mowbite_safety (S2), which alone writes hw/cmd_vel; the logic tags each with its source.
# The logic stops the mower: passes always and ends the drive of the source that moves.
uint8 SOURCE_STOP=0
# Remote control, only while an area is recorded.
uint8 SOURCE_TELEOP=1
# The follower in the map frame (follower/cmd_vel): mowing, transit, drive_to, undocking and docking.
uint8 SOURCE_FOLLOWER=2
# The calibration drive (mowbite_calibration), forward only.
uint8 SOURCE_CALIBRATION=3
# The follower in the station frame (follower/dock_cmd_vel): docking by the dock marker, near the station only.
uint8 SOURCE_DOCK=4
# A step of the setup program that moves the wheels (setup/cmd_vel, setup/motion).
uint8 SOURCE_SETUP=5
uint8 source
geometry_msgs/Twist twist
