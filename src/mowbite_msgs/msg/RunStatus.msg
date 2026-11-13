# logic/run, latched: the run of the schedule from its acceptance until the logic is idle again.

string run_id
uint8 PHASE_NONE=0
uint8 PHASE_ANNOUNCING=1
uint8 PHASE_CHECKING=2
uint8 PHASE_UNDOCKING=3
uint8 PHASE_MOWING=4
uint8 PHASE_ENDED=5
uint8 phase
# With PHASE_ENDED: REFUSED (detail the reason), CANCELED (detail the source), UNDOCKING_FAILED, JOB_COMPLETE, the
# reason of going home (SCHEDULE with detail end or pause, DARK, RAIN, ...), EMERGENCY or STOPPED.
string end_code
string detail
