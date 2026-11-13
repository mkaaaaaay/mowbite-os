# schedule/notice: what the schedule tells mowbite_net, for its pushes.
uint8 KIND_CHANGED = 0
uint8 KIND_LOGGED = 1
uint8 kind
# KIND_LOGGED: the entry of schedule.log (word of x-schedule-log, detail and more as text, at HH:MM or empty).
string what
string detail
string more
string at
