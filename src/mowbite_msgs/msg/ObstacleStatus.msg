# obstacles/status at 10 Hz: what the sensors of the platform see around the mower. Older than 1 s counts as without
# sight. Shown only for now; nothing drives by it.
std_msgs/Header header

# Every sensor of obstacle_sources sees; else reason (of Obstacles.msg, NO_FRAMES for a sensor without messages) and
# the source it is of. True without obstacle sources.
bool sight
string reason
string source

# A confirmed track lies in a stop zone: the id and kind of the nearest; 0 and empty without one.
bool stop
uint32 stop_id
string stop_kind
# A confirmed person or animal lies in the zone to go slow.
bool slow

ObstacleTrack[] tracks
