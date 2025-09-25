#include "horiokart_obstacle_detector_3d/core/intensity_utils.hpp"

#include <cmath>

using namespace obstacle_detector;

double obstacle_detector::compensateIntensity(
  double intensity, const PointXYZ & point, const Eigen::Vector3d & sensor_origin,
  const Eigen::Vector3d & sensor_forward, double distance_ref, double distance_power,
  bool compensate_distance, bool compensate_angle, double angle_min_dot)
{
  double in = intensity;
  if (compensate_distance) {
    double dist = std::hypot(point.x, point.y);
    double factor = std::pow((dist / std::max(1e-6, distance_ref)), distance_power);
    if (factor > 0.0) {
      in = in * std::min(10.0, 1.0 / factor);
    }
  }
  if (compensate_angle) {
    Eigen::Vector3d ray(
      point.x - sensor_origin.x(), point.y - sensor_origin.y(), point.z - sensor_origin.z());
    if (ray.norm() > 1e-9) {
      ray.normalize();
      double dot = std::abs(ray.dot(sensor_forward));
      if (dot < angle_min_dot) {
        in *= (dot / std::max(1e-6, angle_min_dot));
      }
    }
  }
  if (in < 0.0) {
    in = 0.0;
  }
  if (in > 1.0) {
    in = 1.0;
  }
  return in;
}
