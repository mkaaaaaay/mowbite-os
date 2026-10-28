# Based on xbot_msgs/AbsolutePose (OpenMower), MIT License, Copyright (c) 2022 Clemens Elflein. See LICENSE.xbot_monitoring.
# Absolute pose from one sensor (GPS, sensor fusion), fields as in the original.

uint8 SOURCE_GPS=1
uint8 SOURCE_LIGHTHOUSE=2
uint8 SOURCE_SENSOR_FUSION=100

uint16 FLAG_GPS_RTK=1                   # corrections applied
uint16 FLAG_GPS_RTK_FIXED=2             # fixed solution
uint16 FLAG_GPS_RTK_FLOAT=4             # float solution
uint16 FLAG_GPS_DEAD_RECKONING=8        # receiver is dead reckoning

uint16 FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE=1
uint16 FLAG_SENSOR_FUSION_DEAD_RECKONING=8

std_msgs/Header header
uint32 sensor_stamp                     # sensor time, 0 for sensor fusion
uint32 received_stamp                   # time of receipt, 0 for sensor fusion

uint8 source
uint16 flags
uint8 orientation_valid                 # applies to the orientation and vehicle_heading
uint8 motion_vector_valid               # applies to motion_vector and motion_heading
float32 position_accuracy               # m
float32 orientation_accuracy            # rad
geometry_msgs/PoseWithCovariance pose   # x east, y north, z up, meters from the datum
geometry_msgs/Vector3 motion_vector     # m/s
float64 vehicle_heading                 # yaw of the orientation
float64 motion_heading                  # yaw of the motion vector
