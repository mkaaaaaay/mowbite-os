# What the platform driver offers, latched on hw/capabilities and published again when a value changes.

# e.g. "airseekers_tron"
string platform
# Firmware per board as reported by the hardware; an empty value until it is known.
diagnostic_msgs/KeyValue[] firmware

bool blade_speed_control
bool blade_height_control
uint16 blade_height_min_mm
uint16 blade_height_max_mm
# Longest lease hw/blade accepts.
float32 blade_lease_max_s

# The host starts charging over hw/charge; false where the driver or the hardware does it.
bool host_charge_control
# hw/charge can stop charging, and the logic does so before the mower leaves the station. False where the hardware
# charges by itself and offers no way to stop it: undocking then leaves without that step.
bool charge_stop
bool rain_sensor
bool separate_bumpers
bool wheel_ticks
# Nodes that provide imu/data and gps/absolute_pose.
string imu_source
string gnss_source

# What the driver computes speed and turn rate of hw/wheel_odom and hw/cmd_vel with: ticks of the wheels per m (a
# driver that reports speed gives its scale, 1000 for mm) and the distance between the drive wheels (m). The
# calibration drive and the localization compare these with what robot.geometry holds.
float64 wheel_ticks_per_m
float64 wheel_distance

# Limits the driver enforces on hw/cmd_vel; 0 lets nothing move (wheels locked, xCore max_duty 0).
float32 max_linear_speed
float32 max_angular_speed

# How long the hardware keeps the last setpoint without the host, measured on the device; negative while unknown.
float32 failsafe_without_host_s
# Date of the safety measurement of this platform (YYYY-MM-DD), empty without one. Without it the logic stays with
# remote control: no autonomous driving, no blade.
string safety_measurement_date
