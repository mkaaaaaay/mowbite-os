# gnss/ntrip_status, once a second: the NTRIP client (mowbite_ntrip). Never carries the access data.
# off: no access file; connecting; connected: frames arrive; rejected: the caster refused the login, waiting;
# waiting: no connection, next attempt after retry_in s.
string state
# Short reason of the last failure, empty if none.
string last_error
uint32 connects
uint64 bytes
uint64 frames
uint64 crc_errors
# s since the last whole frame, -1 before the first.
float64 last_frame_age
# s until the next attempt while waiting or rejected, otherwise 0.
float64 retry_in
