# One input of the platform, in hw/inputs.

# As the hardware names it, e.g. "GPIO10"; a fixed input of a board has a name of the driver.
string name
# What hw/status reports it as: stop, lift, bumper_left or bumper_right; empty for a line without a role.
string role
# The emergency the hardware latches with it: stop, lift or collision; empty for none.
string reason
# Switched now. While the driver learns: the level of the line is high.
bool active
# In the table of this mower; false for a candidate line the driver only watches while it learns.
bool assigned
# The hardware keeps it switched after the sensor is free, until the emergency is released.
bool held
# The level that counts as switched, and how long it has to stand before the hardware latches (ms).
bool active_high
uint16 delay_ms
