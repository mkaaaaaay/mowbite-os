# hw/dock_marker: where the marker of the docking station is, seen from the mower. Optional; published by the process
# that measures it (reliable, volatile, depth 10): one message per evaluated image while it measures, and in every
# state at least one per REPORT_PERIOD_S. A valid message is the measurement of exactly one image, never an older or
# an estimated one.
float32 REPORT_PERIOD_S=1.0

# The process measures only while asked. Every message on hw/dock_marker_request (std_msgs/Empty, reliable, volatile)
# asks for the next REQUEST_HOLD_S seconds from its arrival; who wants measurements sends one every REQUEST_PERIOD_S.
float32 REQUEST_PERIOD_S=1.0
float32 REQUEST_HOLD_S=3.0

# stamp: when the image was taken, by the host clock; without an image, when the message was made. frame_id base_link.
std_msgs/Header header

# OFF: not asked. STARTING: asked, no image evaluated yet; it ends by itself in MEASURING or FAULT.
# MEASURING: every message is one image, with a measurement (valid) or the reason why it gave none.
# FAULT: asked but unable to measure. It stays until the requests ran out (OFF); the next request starts anew.
uint8 STATE_OFF=0
uint8 STATE_STARTING=1
uint8 STATE_MEASURING=2
uint8 STATE_FAULT=3
uint8 state

# This image gave a measurement; x, y, yaw and covariance mean something only then.
bool valid
# Centre of the marker in base_link, m; its height is left out.
float64 x
float64 y
# Direction the marker faces (the normal out of its front), rad in base_link: a marker that looks straight at the
# origin of base_link has yaw = atan2(-y, -x). Only while yaw_valid, which the process clears where the angle cannot
# be trusted.
float64 yaw
bool yaw_valid
# Covariance of x, y, yaw in this order, row-major (m^2, m rad, rad^2): the spread of this one measurement, without
# the error of the calibration. The entries of yaw are 0 without yaw_valid.
float64[9] covariance

# Why there is no measurement: always set in MEASURING without valid and in FAULT, in OFF when a request would fail
# now, else empty.
# The image: no marker found; it touches the edge of the image; more than one; exposure; sharpness; found, but the fit
# of its pose does not hold.
string REASON_NO_MARKER=NO_MARKER
string REASON_MARKER_CLIPPED=MARKER_CLIPPED
string REASON_MARKER_AMBIGUOUS=MARKER_AMBIGUOUS
string REASON_TOO_DARK=TOO_DARK
string REASON_TOO_BRIGHT=TOO_BRIGHT
string REASON_BLURRED=BLURRED
string REASON_FIT_REJECTED=FIT_REJECTED
# The camera: the reasons of Camera.msg (REASON_NO_SIGNAL also for no image or a broken one).
string reason
# Why, in a few words for the log.
string detail
