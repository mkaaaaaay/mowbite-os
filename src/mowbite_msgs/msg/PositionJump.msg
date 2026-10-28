# The localization moved its estimate onto GPS again, after a gap without fixes or after a run of outliers, by more
# than max_position_accuracy.
uint8 REASON_GAP=1
uint8 REASON_OUTLIERS=2

std_msgs/Header header
uint8 reason
geometry_msgs/Point before
geometry_msgs/Point after
float64 distance
