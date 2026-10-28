# Based on xbot_msgs/xbot_mqtt (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
# Describes one sensor for sensor_infos/json, same fields and values as xbot_msgs/SensorInfo.
uint8 TYPE_STRING=1
uint8 TYPE_DOUBLE=2

uint8 VALUE_DESCRIPTION_UNKNOWN=0
uint8 VALUE_DESCRIPTION_TEMPERATURE=1
uint8 VALUE_DESCRIPTION_VELOCITY=2
uint8 VALUE_DESCRIPTION_ACCELERATION=3
uint8 VALUE_DESCRIPTION_VOLTAGE=4
uint8 VALUE_DESCRIPTION_CURRENT=5
uint8 VALUE_DESCRIPTION_PERCENT=6
uint8 VALUE_DESCRIPTION_DISTANCE=7
uint8 VALUE_DESCRIPTION_RPM=8

string sensor_id
string sensor_name
uint8 value_type
uint8 value_description
string unit
bool has_min_max
float64 min_value
float64 max_value
bool has_critical_low
float64 lower_critical_value
bool has_critical_high
float64 upper_critical_value
