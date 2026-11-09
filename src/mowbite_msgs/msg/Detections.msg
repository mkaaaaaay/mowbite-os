# vision/detections: the objects of one inference of a camera, best score first; empty when nothing was found.
# header.stamp is when the frame was taken by the host clock.
std_msgs/Header header
string camera_id
# Size of the camera image the boxes refer to.
uint32 width
uint32 height
# Time of the inference on the NPU.
float32 infer_ms
Detection[] detections
