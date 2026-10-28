# Based on xbot_msgs/xbot_mqtt (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
# What the app shows from robot_state/json, sent by mowbite_logic about once a second.
string current_state
string current_sub_state
int16 current_area
int16 current_path
int16 current_path_index
float32 current_action_progress
# 0..1
float32 battery_percentage
float32 gps_percentage
bool emergency
# Not in OpenMower: bits as in EMERGENCY events (EmergencyReason), 0 without emergency.
uint16 emergency_reason
bool is_charging
bool rain_detected

float64 pose_x
float64 pose_y
float64 pose_heading
float32 pos_accuracy
float32 heading_accuracy
bool heading_valid
