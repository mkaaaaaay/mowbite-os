# calibration/status of mowbite_calibration, twice a second: what keeps a calibration drive from starting where the
# mower stands (room, sensors, map), and the lane it would take. The logic answers calibration.check and
# calibration.start from it.
builtin_interfaces/Time stamp
# Empty when a drive could start.
CalibrationProblem[] problems
# The lane: heading of its first line (rad, map frame) and length (m), 0 without room; the strip it needs (m), without
# room the one of the shortest lane.
float64 heading
float64 lane_length
float64 strip_length
float64 strip_width
# A drive on that lane: its rounds and the time it should take (s) by the expected time of its phases, without the
# evaluation; 0 without room.
int32 rounds
float64 expected_s
# A drive runs.
bool running
