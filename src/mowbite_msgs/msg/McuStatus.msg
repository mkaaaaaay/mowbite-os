builtin_interfaces/Time stamp

# Sensor info from the MCU within telemetry_timeout.
bool telemetry_ok
# Lift, stop, bumper or a host request latch the emergency until it is released.
bool emergency_latched
# Firmware check on and no matching version frame since the port opened: motion and blade stay blocked, a release
# does not lift it.
bool firmware_incompatible
bool lift
bool stop
bool bumper
bool bumper_left
bool bumper_right
bool rain
# 1 small, 2 large deck
uint8 cutter_size

bool moving
bool blade_on
# Debounced dock contact, charge is enabled while set.
bool dock_contact

bool battery_ok
float32 battery_voltage
# Positive while discharging.
float32 battery_current
uint8 battery_percentage
int8 battery_temperature
bool battery_dock
uint32 battery_error_bits

MotorState cutter
MotorState left
MotorState right

string cutter_version
string chassis_version
string bms_version

uint64 frames_ok
uint64 frame_errors
