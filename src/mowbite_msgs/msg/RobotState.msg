# bridge/robot_state, once a second from the logic; the bridge sends it on as robot_state/json.
string current_state
string current_sub_state
# 0..1
float32 battery_percentage
# 0..1, 0 without a recent pose or heading.
float32 gps_percentage
bool emergency
# Bits as in EMERGENCY events, 0 without emergency.
uint32 emergency_reason
bool is_charging
bool rain_detected
# While mowing: the area, the path of its plan and the pose in it; otherwise -1.
int32 current_area
int32 current_path
int32 current_path_index
# While mowing: the place of the area in the areas of the job and their number; otherwise -1.
int32 task_index
int32 task_count

# The pose of the mower for the app.
float64 pose_x
float64 pose_y
float64 pose_heading
float32 pos_accuracy
float32 heading_accuracy
bool heading_valid
