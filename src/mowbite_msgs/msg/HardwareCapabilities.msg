# What the platform driver offers, latched on hw/capabilities and published again when a value changes.

# e.g. "airseekers_tron"
string platform
# Firmware per board as reported by the hardware; an empty value until it is known.
diagnostic_msgs/KeyValue[] firmware

bool blade_speed_control
# The board takes a new height in hw/blade while the blade runs, without stopping it.
bool blade_height_control
uint16 blade_height_min_mm
uint16 blade_height_max_mm
# Height the driver cuts at when hw/blade asks for 0; 0 without blade_height_control.
uint16 blade_height_default_mm
# Longest lease hw/blade accepts.
float32 blade_lease_max_s
# hw/blade may switch the blade on (allow_blade of the driver); false: it refuses every on.
bool blade_allowed

# The host starts charging over hw/charge; false where the driver or the hardware does it.
bool host_charge_control
# hw/charge can stop charging, and the logic does so before the mower leaves the station. False where the hardware
# charges by itself and offers no way to stop it: undocking then leaves without that step.
bool charge_stop
bool rain_sensor
# Sources of hw/obstacles, e.g. the camera id "stereo"; empty where the platform has no obstacle sensor.
string[] obstacle_sources
# Highest obstacle_mode the obstacle sensors are proven for on this platform: 1 shows them, 2 may stop the mower,
# 3 may drive around them. 0 without obstacle sources.
uint8 obstacle_mode_max
bool separate_bumpers
bool wheel_ticks
# Nodes that provide imu/data and gps/absolute_pose.
string imu_source
string gnss_source
# receiver_time_ms of gps/absolute_pose is the UTC time of day of the receiver; false where it is a clock of the board.
bool gnss_receiver_time

# What the driver computes speed and turn rate of hw/wheel_odom and hw/cmd_vel with: ticks of the wheels per m (a
# driver that reports speed gives its scale, 1000 for mm) and the distance between the drive wheels (m). The
# calibration drive and the localization compare these with what robot.geometry holds.
float64 wheel_ticks_per_m
float64 wheel_distance

# Limits the driver enforces on hw/cmd_vel; 0 lets nothing move (wheels locked, xCore max_duty 0).
float32 max_linear_speed
float32 max_angular_speed

# Releasing the emergency with the STOP button (hw/button): free this long, then held this long and let go before
# max_hold (s). 0 where the driver offers no such release.
float32 stop_release_free_s
float32 stop_release_hold_s
float32 stop_release_max_hold_s
# How long a button other than STOP is held for HELD on hw/button (s); 0 without such buttons.
float32 button_hold_s

# How long the hardware keeps the last setpoint without the host, measured on the device; negative while unknown.
float32 failsafe_without_host_s
# Longest silence hw/silence takes (s); 0 where the driver offers no hw/silence.
float32 silence_max_s
# Longest silence hw/silence takes with last_setpoint (s); 0 where the driver does not offer it.
float32 silence_last_setpoint_max_s
# The driver offers hw/end.
bool driver_end
# Date of the safety measurement of this platform (YYYY-MM-DD), empty without one. Without it the logic stays with
# remote control: no autonomous driving, no blade.
string safety_measurement_date
# Whether the firmware the boards report may drive and turn the blade, the rule of the driver alone: empty until every
# board reported its firmware; listed (the list of the driver), measured (an owner's measurement of exactly these
# firmwares on this device, measured_firmware of the driver), older (a board older than every listed one: update it
# first, no measurement), none (blocked until listed or measured). Only listed and measured let the mower drive.
string firmware_source
# sha256 (64 hex digits) of an id of this device the driver reads; empty where it has none. A measured firmware holds
# only with the hash it was measured with.
string device_hash
