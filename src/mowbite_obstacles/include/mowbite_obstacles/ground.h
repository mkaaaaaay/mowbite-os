// Where a box of a camera stands on the ground, in base_link, without ROS. The platform turns pixels into rays with
// its own lens model; everything here works on rays.
#ifndef MOWBITE_OBSTACLES_GROUND_H
#define MOWBITE_OBSTACLES_GROUND_H

namespace mowbite_obstacles {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

// A direction in the frame of the camera as x/z and y/z: z along the view, x to the right, y down.
struct Ray {
  double x = 0.0;
  double y = 0.0;
};

// The optical center of the camera in base_link (m) and its orientation as roll, pitch and yaw about the axes of
// base_link (rad, applied roll first). All zero: it looks along x of base_link, the image upright; pitch > 0 looks
// down.
struct Mount {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
};

// The ground as the plane normal . p = offset in base_link; flat ground lies at z = -wheel_radius.
struct Plane {
  Vec3 normal{0.0, 0.0, 1.0};
  double offset = 0.0;
};
Plane FlatGround(double wheel_radius);

// A position on the ground in base_link with its covariance (m, m^2).
struct Spot {
  double x = 0.0;
  double y = 0.0;
  double cov_xx = 0.0;
  double cov_xy = 0.0;
  double cov_yy = 0.0;
};

// The direction of a ray in base_link, not normalized.
Vec3 RayInBase(const Mount& mount, const Ray& ray);
// The ray that sees a point of base_link; false where it lies behind the camera.
bool RayOf(const Mount& mount, const Vec3& point, Ray* ray);
// Where a ray meets the ground; false where it does not go down to it ahead of the camera.
bool HitGround(const Mount& mount, const Plane& ground, const Ray& ray, Vec3* point);

// A box by the rays of the middle of its lower edge, of its two lower corners and of the middle of its upper edge.
struct BoxRays {
  Ray bottom;
  Ray bottom_left;
  Ray bottom_right;
  Ray top;
  // The box touches the lower edge of the image: its lower edge is not where it stands.
  bool cut_bottom = false;
};

// How far a ray may be off (rad, about one or two pixels of a box edge over the focal length) and how well the mount
// is known.
struct Errors {
  double ray = 0.0;
  double pitch = 0.0;
  double height = 0.0;
};

enum class GroundResult {
  kOk,
  // The lower edge looks at or above the horizon.
  kAboveHorizon,
  // It meets the ground farther than the range given.
  kTooFar,
};

// The footprint of a box on the ground: its middle one radius behind the point under its lower edge, the radius half
// the width between its lower corners, the height up to its upper edge where it stands (NaN where that ray misses),
// and the spread of the footprint from errors. With cut_bottom the footprint is a far bound: the thing stands nearer.
struct Footprint {
  GroundResult result = GroundResult::kAboveHorizon;
  Spot spot;
  double radius = 0.0;
  double height = 0.0;
  // From the camera to the footprint on the ground (m).
  double range = 0.0;
};
Footprint FootprintOf(const Mount& mount, const Plane& ground, const BoxRays& box, const Errors& errors,
                      double range_max);

// A point stereo depth gives: the ray and the distance along the view (z in the frame of the camera, m) with its
// spread (m) and that of the ray (rad).
Spot StereoSpot(const Mount& mount, const Ray& ray, double depth, double depth_sigma, double ray_sigma);

// The two positions of one thing fused by their covariances, and whether they agree within gate (squared
// Mahalanobis distance; 9.21 holds 99 % of agreeing pairs). Where they disagree the nearer to base_link counts, its
// covariance widened by their difference.
struct Fused {
  Spot spot;
  bool agrees = false;
};
Fused Fuse(const Spot& a, const Spot& b, double gate);

// The sector of the ground a camera sees, as Obstacles.msg has it: around the camera, from the angle of the ray at
// the right edge of the image to that at the left, from where the ray at the middle of the lower edge meets the ground
// to range_max. False where that ray misses the ground.
struct View {
  double x = 0.0;
  double y = 0.0;
  double angle_left = 0.0;
  double angle_right = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
};
bool ViewOf(const Mount& mount, const Plane& ground, const Ray& left_edge, const Ray& right_edge,
            const Ray& bottom_middle, double range_max, View* view);

}  // namespace mowbite_obstacles

#endif  // MOWBITE_OBSTACLES_GROUND_H
