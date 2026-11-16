# One turn on the spot of mowbite_follower, on follower/spot_turn when it ended: what the commands asked for, what
# the wheels reported and what the pose shows. turned / wheels below 1 is slip.
uint8 KIND_START=0
uint8 KIND_CORNER=1
uint8 KIND_REALIGN=2
uint8 KIND_GOAL=3

builtin_interfaces/Time stamp
uint8 kind
# The frame of the path, as in FollowPath.action.
uint8 frame
float32 seconds
# rad, counterclockwise positive: commanded, by the odometry of the wheels (NaN without it), by the heading.
float32 commanded
float32 wheels
float32 turned
# m base_link got from the place the turn began at, at most and at its end, and m driven to hold that place.
float32 slide
float32 end_slide
float32 hold
