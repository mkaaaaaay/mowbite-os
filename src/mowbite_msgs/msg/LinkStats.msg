# hw/link_stats: what the driver read from its board since it started, about once a second. The setup program checks
# the link with it while it measures firmware.
builtin_interfaces/Time stamp
# Frames with a wrong checksum, frames of a known module with a wrong length, frames of a module the driver does not
# know, all since the start of the driver.
uint64 checksum_errors
uint64 length_errors
uint64 unknown_frames
# Frames per module the driver knows since its start, by name (the Tron: sensor_info, speed, version, battery, bms,
# motors); rates come from two of these.
string[] modules
uint64[] frames
# Ids of unknown modules and how often each came, at most 16.
uint16[] unknown_ids
uint64[] unknown_counts
# The last clear setup/firmware_measure asked for that went to the board, and when; 0 before the first.
uint32 clear_seq
builtin_interfaces/Time clear_stamp
