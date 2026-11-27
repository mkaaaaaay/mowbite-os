# setup/motion: a step of the setup program (mowbite_setup) that moves the wheels or turns the blade, at 4 Hz while it
# runs, at once when blade changes, and once with active false at its end. The logic passes setup/cmd_vel as
# SOURCE_SETUP only while it is active; mowbite_safety caps SOURCE_SETUP to these limits and refuses it without a
# recent one.
builtin_interfaces/Time stamp
bool active
# The step, as in setup.json.
string step
# The step runs with the mower jacked up.
bool raised
# Limits of the step: m/s of the body and of each wheel, rad/s.
float32 max_linear
float32 max_angular
# The blade: off, on with a lease renewed by the logic, or on with one lease the logic does not renew (the driver has
# to switch it off). Only in SETUP, without emergency and off the station; the logic switches it off when SETUP ends.
uint8 BLADE_OFF = 0
uint8 BLADE_ON = 1
uint8 BLADE_ONCE = 2
uint8 blade
# Percent of full speed, 0 = driver default; height in mm, 0 = driver default; the lease (s), at most
# blade_lease_max_s of hw/capabilities.
uint8 blade_percent
uint16 blade_height_mm
float32 blade_lease_s
# After an emergency of only the inputs of the board (STOP, LIFT, COLLISION) the logic keeps the blade order this long
# (s), so that the board alone has to stop the blade; 0: off at once. Only a step that measures firmware with the blade
# off the mower (confirmed blade_removed) sets it.
float32 blade_input_hold_s
