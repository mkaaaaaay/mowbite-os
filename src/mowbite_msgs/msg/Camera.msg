# One camera of the platform on hw/cameras, published by the process that owns it, latched (transient_local, depth at
# least the number of cameras of that process) and again whenever a field changes. The OS keeps the list by id.

# Stable within a platform, e.g. "oa_left", "stereo"; also part of the topic camera/<id>/jpeg.
string id
# Shown in the app.
string name
# Name of the owning process as given to stack.own_process; after <process>_exit.json its cameras count as not
# available (NO_SIGNAL) until it publishes again.
string process

string KIND_OBSTACLE=obstacle
string KIND_STEREO=stereo
string KIND_REAR=rear
string kind

# Where it looks, for the switch of the app; empty when unknown or not fixed.
string POSITION_FRONT=front
string POSITION_REAR=rear
string POSITION_LEFT=left
string POSITION_RIGHT=right
string position

# Size of the image a viewer gets.
uint32 width
uint32 height
# Boxes of the camera AI are burnt into the image and come on vision/detections.
bool detections

# mjpeg: JPEG frames on camera/<id>/jpeg (sensor_msgs/CompressedImage), sent only while someone subscribes.
# webrtc: H.264 at `source`, an RTSP address the OS pulls from on demand.
string PROTOCOL_MJPEG=mjpeg
string PROTOCOL_WEBRTC=webrtc
string[] protocols
string source

# False with one of the reasons below while the camera cannot stream; the reason is empty while it can. The OS refuses
# a third viewer with CAMERA_BUSY on its own.
bool available
string REASON_LOCKED_AFTER_CRASH=LOCKED_AFTER_CRASH
string REASON_NOT_SET_UP=NOT_SET_UP
string REASON_BLADE_RUNNING=BLADE_RUNNING
string REASON_TOO_HOT=TOO_HOT
string REASON_NO_SIGNAL=NO_SIGNAL
# hw/status older than 1 s; BLADE_RUNNING only while the blade really runs.
string REASON_DRIVER_MISSING=DRIVER_MISSING
# The self-test after the start is pending or running; it passes on its own.
string REASON_SELF_TEST=SELF_TEST
string reason
# Why, in a few words for the log and the app ("model missing", "SoC 81 C").
string detail
