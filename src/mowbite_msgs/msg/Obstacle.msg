# One obstacle on the ground as a sensor of the platform sees it, in base_link.

# What it is. The platform maps the classes of its model to these in its own file; an unknown class is OBJECT.
# UNKNOWN: only geometry (depth), no class.
string KIND_PERSON=PERSON
string KIND_ANIMAL=ANIMAL
string KIND_OBJECT=OBJECT
string KIND_UNKNOWN=UNKNOWN
string kind
# As the model names it, for the app only; empty for UNKNOWN.
string class_name
# Score of the model, 0..1; 1 for UNKNOWN.
float32 score

# Middle of its footprint in base_link (m), its radius on the ground (m) and its height (m, NaN when unknown).
float32 x
float32 y
float32 radius
float32 height
# Covariance of x and y in base_link (m^2).
float32 cov_xx
float32 cov_xy
float32 cov_yy
# The box touches the lower edge of the image: the footprint is nearer than x, y.
bool cut_bottom

# What the position comes from: the ground plane under the lower edge of the box, stereo depth, or both. With both,
# agrees tells whether they fit within their errors; x, y is then the fused position.
string FROM_GROUND=GROUND
string FROM_STEREO=STEREO
string FROM_BOTH=BOTH
string position_from
bool agrees

# Box in pixels of the image on hw/obstacle_images with the same stamp, top left corner; 0 for w without one.
float32 box_x
float32 box_y
float32 box_w
float32 box_h
