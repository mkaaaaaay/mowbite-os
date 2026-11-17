# setup/motion: a step of the setup program (mowbite_setup) that moves the wheels, at 4 Hz while it runs and once
# with active false at its end. The logic passes setup/cmd_vel as SOURCE_SETUP only while it is active; mowbite_safety
# caps SOURCE_SETUP to these limits and refuses it without a recent one.
builtin_interfaces/Time stamp
bool active
# The step, as in setup.json.
string step
# The step runs with the mower jacked up.
bool raised
# Limits of the step: m/s of the body and of each wheel, rad/s.
float32 max_linear
float32 max_angular
