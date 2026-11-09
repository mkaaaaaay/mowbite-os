# A press of a button on the mower as the platform driver recognised it, on hw/button (reliable, volatile, never
# replayed). A driver without such buttons publishes nothing.
builtin_interfaces/Time stamp

# The red STOP on top of the mower.
uint8 BUTTON_STOP=1
uint8 button

# STOP while the emergency is latched, after it was free for stop_release_free_s of hw/capabilities: HELD once it was
# held for stop_release_hold_s, LONG once it was let go after that and before stop_release_max_hold_s, CANCELED when a
# held press ends without LONG (too long, lift changed, a new reason latched, telemetry or IMU stale, released already).
uint8 PRESS_HELD=1
uint8 PRESS_LONG=2
uint8 PRESS_CANCELED=3
uint8 press

# With STOP LONG: latch_count of the driver after the press began; a release over hw/emergency with it is refused with
# LATCH_CHANGED once another reason latched since. 0 otherwise.
uint32 latch_count
