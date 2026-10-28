# One motor as reported by the MCU. ok is false when the motor report is missing or stale.
bool ok
int16 rpm
# Cutter in A; drive boards scaled by the drive_current_scale parameter (unit not verified).
float32 current
int8 temperature
# Status code of the board: values below zero are faults, otherwise idle 0, running 1, locking 2
int8 status
