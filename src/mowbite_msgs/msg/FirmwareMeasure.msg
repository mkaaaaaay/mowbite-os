# setup/firmware_measure: a step of the setup program (mowbite_setup) that measures firmware the driver reports with
# firmware_source none, at 4 Hz while it runs and once with active false at its end. While a recent one (0.5 s) names
# exactly the firmware and device_hash the driver has, the driver lets the step drive and turn the blade within its own
# limits for the measurement (the Tron: 0.1 m/s); every other reason of the emergency works as always. Without it, or
# for other firmware, motion and blade stay blocked (FIRMWARE_INCOMPATIBLE).
builtin_interfaces/Time stamp
bool active
# The step, as in setup.json.
string step
# The firmware the step measures, board by board as firmware of hw/capabilities names it, and device_hash.
diagnostic_msgs/KeyValue[] firmware
string device_hash
# Raised by one for each clear the step asks for: the driver sends one clear of the latches to the board while the
# wheels and the blade stand, and names it in clear_seq of hw/link_stats.
uint32 clear_seq
