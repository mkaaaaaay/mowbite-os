# State of the mower hardware, published by the platform driver on hw/status at 10 Hz.
builtin_interfaces/Time stamp

# Emergency reason bits, numbered like EmergencyReason of OpenMower; own bits start at 1 << 13 (13 to 15 are kept for
# TILT, FENCE and STUCK, which no node sets yet).
uint32 REASON_LATCH = 1
uint32 REASON_TIMEOUT_INPUTS = 2
uint32 REASON_STOP = 4
uint32 REASON_LIFT = 8
uint32 REASON_COLLISION = 32
uint32 REASON_HIGH_LEVEL = 128
uint32 REASON_FIRMWARE_INCOMPATIBLE = 4096
# No IMU sample to forward to the MCU, whose tilt detection depends on it.
uint32 REASON_IMU_TIMEOUT = 65536
# The blade still turns after it was switched off.
uint32 REASON_MOWER_NOT_STOPPED = 131072

# Motion and blade are blocked by the driver: latched, stale telemetry or incompatible firmware.
bool emergency
# Stays until released over hw/emergency.
bool emergency_latched
# Reason bits while emergency is set, 0 otherwise. A cause that latched stays in it until the release.
uint32 emergency_reason
# Sensor data from the hardware within the timeout of the driver.
bool telemetry_ok

# Live sensor flags, false without telemetry.
bool lift
bool stop
bool bumper
bool bumper_left
bool bumper_right
bool rain
# Debounced contact with the charging station.
bool dock_contact
# The contact as the last report had it, not debounced: a mower driving onto the station stops at the first one.
bool dock_contact_raw

# Wheels commanded to move.
bool moving

uint8 BLADE_OFF_NONE = 0
uint8 BLADE_OFF_REQUEST = 1
uint8 BLADE_OFF_LEASE_EXPIRED = 2
uint8 BLADE_OFF_BLOCKED = 3
uint8 BLADE_OFF_DOCK = 4
uint8 BLADE_OFF_RECONNECT = 5
# Blade commanded on, and why it went off the last time.
bool blade_on
uint8 blade_off_reason

uint8 BATTERY_SOURCE_NONE = 0
uint8 BATTERY_SOURCE_BMS = 1
uint8 BATTERY_SOURCE_ESTIMATE = 2
# NONE while the battery report is missing or stale; the battery fields are 0 then.
uint8 battery_source
float32 battery_voltage
# Positive while discharging.
float32 battery_current
# 0-100
uint8 battery_percentage
int8 battery_temperature
# Raw error bits of the BMS, 0 without one.
uint32 battery_error_bits

MotorState blade_motor
MotorState left_motor
MotorState right_motor

# Messages on the hardware link, and those dropped for checksum, length or framing errors.
uint64 link_messages_ok
uint64 link_errors
