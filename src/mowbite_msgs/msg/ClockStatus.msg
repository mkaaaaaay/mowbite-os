# time/clock, latched, on every change and every 60 s: whether the wall clock can be trusted for date, weekday and sun
# (mowbite_time). reason while not ok: never_synced, clock_jump or gnss_mismatch.
bool ok
string reason
