# obstacles/action: what the mower did about a track of obstacles/status, for its entry in vision.obstacles.list.
uint32 track_id
# slowed, would_stop (obstacle_mode show, it would have stopped), stopped or avoided.
string action
# With avoided: the circle the mower keeps out of around it (m, 0 once the circle is gone), the m of its job's paths
# left unmowed for it, and the area of the job.
float32 keep_out_r
float32 left_out
string area_id
string area_name
