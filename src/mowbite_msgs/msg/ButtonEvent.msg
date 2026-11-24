# A press of a button on the mower as the platform driver recognised it, on hw/button (reliable, volatile, never
# replayed). A driver without such buttons publishes nothing. Several nodes may publish here, each for its own buttons.
builtin_interfaces/Time stamp

# The red STOP on top of the mower.
uint8 BUTTON_STOP=1
# Buttons of the mower for going home, starting and switching off.
uint8 BUTTON_HOME=2
uint8 BUTTON_START=3
uint8 BUTTON_POWER=4
uint8 button

# STOP while the emergency is latched, after it was free for stop_release_free_s of hw/capabilities: HELD once it was
# held for stop_release_hold_s, LONG once it was let go after that and before stop_release_max_hold_s, CANCELED when a
# held press ends without LONG (too long, lift rose, a new reason latched, telemetry or IMU stale, released already).
# Any other button: SHORT when let go before button_hold_s of hw/capabilities, HELD once when it is still held at
# button_hold_s and nothing when let go after that; nothing for a press while another button is held.
uint8 PRESS_HELD=1
uint8 PRESS_LONG=2
uint8 PRESS_CANCELED=3
uint8 PRESS_SHORT=4
uint8 press

# With STOP LONG: latch_count of the driver after the press began; a release over hw/emergency with it is refused with
# LATCH_CHANGED once another reason latched since. 0 otherwise.
uint32 latch_count
