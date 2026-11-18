# hw/obstacle_images: the whole image of a frame of hw/obstacles that found something, with the same header.stamp and
# source; the boxes are in its pixels. Sent at most once a second and only while obstacles/photos_wanted is true.
std_msgs/Header header
string source
uint32 width
uint32 height
# JPEG, at most 100 KB.
uint8[] jpeg
