#include "horiokart_obstacle_detector_3d/node_helpers/point_extractor.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <Eigen/Dense>
#include <functional>
#include <sensor_msgs/point_cloud2_iterator.hpp>
// intensity / color helpers
#include "horiokart_obstacle_detector_3d/core/color_utils.hpp"
#include "horiokart_obstacle_detector_3d/core/intensity_utils.hpp"

namespace obstacle_detector_node_helpers {
using obstacle_detector::PointXYZ;

void extractPointsAndMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf,
    std::vector<obstacle_detector::PointXYZ> &pts,
    std::unordered_map<std::string, obstacle_detector::PointXYZ> &dummy_meta,
    std::unordered_map<std::string, ColorInfo> &point_meta, bool &has_rgb_field,
    bool &has_intensity_field, double roi_x_min, double roi_x_max,
    double roi_y_min, double roi_y_max, double roi_z_min, double roi_z_max,
    bool use_rgb, bool use_intensity, double intensity_distance_ref,
    double intensity_distance_power, bool intensity_compensate_distance,
    bool intensity_compensate_angle, double intensity_angle_min_dot,
    const std::function<Eigen::Vector3d(const tf2::TimePoint &)>
        &getSensorForward,
    const std::function<geometry_msgs::msg::TransformStamped(
        const tf2::TimePoint &)> &getSensorTransform,
    const std::function<std::string(double, double, double)> &make_key,
    const std::function<std::vector<double>(const std::string &,
                                            const std::vector<double> &)>
        &get_param_double_vec) {
  size_t approx = static_cast<size_t>(cloud_in_tf->width) *
                  static_cast<size_t>(cloud_in_tf->height);
  pts.reserve(approx > 0 ? approx : 1024);
  has_rgb_field = false;
  has_intensity_field = false;
  for (const auto &f : cloud_in_tf->fields) {
    if (f.name == "rgb" || f.name == "rgba") {
      has_rgb_field = true;
    }
    if (f.name == "intensity") {
      has_intensity_field = true;
    }
  }
  sensor_msgs::PointCloud2ConstIterator<float> itx(*cloud_in_tf, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ity(*cloud_in_tf, "y");
  sensor_msgs::PointCloud2ConstIterator<float> itz(*cloud_in_tf, "z");
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_rgb;
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> it_intensity;
  if (has_rgb_field)
    it_rgb = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
        *cloud_in_tf, "rgb");
  if (has_intensity_field)
    it_intensity =
        std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(
            *cloud_in_tf, "intensity");

  for (; itx != itx.end(); ++itx, ++ity, ++itz) {
    obstacle_detector::PointXYZ p{*itx, *ity, *itz};
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      if (has_rgb_field)
        ++(*it_rgb);
      if (has_intensity_field)
        ++(*it_intensity);
      continue;
    }
    if (p.x < roi_x_min || p.x > roi_x_max || p.y < roi_y_min ||
        p.y > roi_y_max || p.z < roi_z_min || p.z > roi_z_max) {
      if (has_rgb_field)
        ++(*it_rgb);
      if (has_intensity_field)
        ++(*it_intensity);
      continue;
    }
    ColorInfo ci;
    if (has_rgb_field && it_rgb) {
      ci.has_rgb = true;
      ci.rgb = **it_rgb;
    }
    if (has_intensity_field && it_intensity) {
      ci.has_intensity = true;
      ci.intensity = **it_intensity;
    }
    // intensity normalization parameters via callback
    std::vector<double> inorm = get_param_double_vec(
        "intensity_normalize_range", std::vector<double>{0.0, 1.0});
    double i_min = inorm.size() > 0 ? inorm[0] : 0.0;
    double i_max = inorm.size() > 1 ? inorm[1] : 1.0;
    if (ci.has_intensity) {
      double raw = ci.intensity;
      double in = (raw - i_min) / (std::max(1e-6, (i_max - i_min)));
      tf2::TimePoint when = tf2::TimePoint(
          std::chrono::seconds(cloud_in_tf->header.stamp.sec) +
          std::chrono::nanoseconds(cloud_in_tf->header.stamp.nanosec));
      Eigen::Vector3d sensor_fwd = getSensorForward(when);
      Eigen::Vector3d sensor_origin(0.0, 0.0, 0.0);
      try {
        auto tfst = getSensorTransform(when);
        sensor_origin.x() = tfst.transform.translation.x;
        sensor_origin.y() = tfst.transform.translation.y;
        sensor_origin.z() = tfst.transform.translation.z;
      } catch (const tf2::TransformException &) {
        // ignore
      }
      // Use the existing compensateIntensity in core/intensity_utils
      in = obstacle_detector::compensateIntensity(
          in, p, sensor_origin, sensor_fwd, intensity_distance_ref,
          intensity_distance_power, intensity_compensate_distance,
          intensity_compensate_angle, intensity_angle_min_dot);
      ci.intensity = static_cast<float>(in);
    }
    point_meta.emplace(make_key(p.x, p.y, p.z), ci);
    if (has_rgb_field)
      ++(*it_rgb);
    if (has_intensity_field)
      ++(*it_intensity);
    pts.push_back(p);
  }
}

} // namespace obstacle_detector_node_helpers
