#ifndef TRON_VISION_LENS_MODEL_H
#define TRON_VISION_LENS_MODEL_H

namespace tron_vision {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

// Pinhole camera with the lens distortion of the plumb_bob model. A direction ("ray") is x/z and y/z in the frame of
// the camera: z along the view, x to the right, y down.
struct LensModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;
};

// Where a direction lands in the image, in pixels; pixel centers have whole numbers.
Point2 ToPixel(const LensModel& lens, const Point2& ray);

// Direction of a pixel position, the distortion removed. False where the model cannot be inverted.
bool ToRay(const LensModel& lens, const Point2& pixel, Point2& ray);

}  // namespace tron_vision

#endif  // TRON_VISION_LENS_MODEL_H
