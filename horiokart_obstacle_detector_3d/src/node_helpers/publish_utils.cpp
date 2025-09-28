#include "horiokart_obstacle_detector_3d/node_helpers/publish_utils.hpp"

#include <algorithm>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace obstacle_detector_node_helpers {
using obstacle_detector::PointXYZ;

void publishConfidenceCloud(
    const obstacle_detector::GridHeightMap &grid,
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_confidence_cloud,
    double roi_x_min, double roi_y_min, double grid_cell_size) {
  if (!(pub_confidence_cloud &&
        pub_confidence_cloud->get_subscription_count() > 0))
    return;
  std::vector<PointXYZ> cells_pts;
  std::vector<float> cells_conf;
  for (int ix = 0; ix < grid.rows(); ++ix) {
    for (int iy = 0; iy < grid.cols(); ++iy) {
      obstacle_detector::GridCell c;
      if (!grid.getCell(ix, iy, c) || !c.has_observation)
        continue;
      double cx = roi_x_min + (ix + 0.5) * grid_cell_size;
      double cy = roi_y_min + (iy + 0.5) * grid_cell_size;
      double cz = c.height_median;
      cells_pts.push_back(PointXYZ{static_cast<float>(cx),
                                   static_cast<float>(cy),
                                   static_cast<float>(cz)});
      cells_conf.push_back(static_cast<float>(c.confidence));
    }
  }
  if (!cells_pts.empty()) {
    auto out_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out_cloud->header = cloud_in_tf->header;
    out_cloud->header.frame_id = cloud_in_tf->header.frame_id;
    out_cloud->height = 1;
    out_cloud->width = static_cast<uint32_t>(cells_pts.size());
    out_cloud->is_bigendian = false;
    out_cloud->is_dense = true;
    out_cloud->fields.clear();
    sensor_msgs::msg::PointField f;
    f.name = "x";
    f.offset = 0;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    f.name = "y";
    f.offset = 4;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    f.name = "z";
    f.offset = 8;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    f.name = "confidence";
    f.offset = 12;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    int field_count = 4;
    out_cloud->point_step = 4 * field_count;
    out_cloud->row_step = out_cloud->point_step * out_cloud->width;
    out_cloud->data.assign(out_cloud->row_step * out_cloud->height, 0);
    sensor_msgs::PointCloud2Iterator<float> ox(*out_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(*out_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(*out_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> oc(*out_cloud, "confidence");
    for (size_t i = 0; i < cells_pts.size(); ++i, ++ox, ++oy, ++oz, ++oc) {
      *ox = cells_pts[i].x;
      *oy = cells_pts[i].y;
      *oz = cells_pts[i].z;
      *oc = cells_conf[i];
    }
    pub_confidence_cloud->publish(*out_cloud);
  }
}

void publishObstacleCloudAndScan(
    const std::vector<obstacle_detector::PointXYZ> &obstacle_pts,
    const std::unordered_map<std::string, obstacle_detector::ColorInfo>
        &point_meta,
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pub_obstacle_cloud,
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan,
    bool use_rgb, bool use_intensity,
    const std::function<std::string(double, double, double)> &make_key,
    const std::function<std::vector<double>(const std::string &,
                                            const std::vector<double> &)>
        &get_param_double_vec) {
  if (!obstacle_pts.empty()) {
    auto out_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out_cloud->header = cloud_in_tf->header;
    out_cloud->header.frame_id = cloud_in_tf->header.frame_id;
    out_cloud->height = 1;
    out_cloud->width = static_cast<uint32_t>(obstacle_pts.size());
    out_cloud->is_bigendian = false;
    out_cloud->is_dense = true;
    out_cloud->fields.clear();
    sensor_msgs::msg::PointField f;
    f.name = "x";
    f.offset = 0;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    f.name = "y";
    f.offset = 4;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    f.name = "z";
    f.offset = 8;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    out_cloud->fields.push_back(f);
    int field_count = 3;
    bool include_rgb = use_rgb && ([&cloud_in_tf]() {
                         for (const auto &ff : cloud_in_tf->fields) {
                           if (ff.name == "rgb" || ff.name == "rgba")
                             return true;
                         }
                         return false;
                       }());
    bool include_intensity = use_intensity && ([&cloud_in_tf]() {
                               for (const auto &ff : cloud_in_tf->fields) {
                                 if (ff.name == "intensity")
                                   return true;
                               }
                               return false;
                             }());
    if (include_rgb) {
      f.name = "rgb";
      f.offset = 4 * field_count;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      out_cloud->fields.push_back(f);
      field_count += 1;
    }
    if (include_intensity) {
      f.name = "intensity";
      f.offset = 4 * field_count;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      f.count = 1;
      out_cloud->fields.push_back(f);
      field_count += 1;
    }
    out_cloud->point_step = 4 * field_count;
    out_cloud->row_step = out_cloud->point_step * out_cloud->width;
    out_cloud->data.assign(out_cloud->row_step * out_cloud->height, 0);
    sensor_msgs::PointCloud2Iterator<float> ox(*out_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(*out_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(*out_cloud, "z");
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> orgb;
    std::unique_ptr<sensor_msgs::PointCloud2Iterator<float>> oint;
    if (include_rgb)
      orgb = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(
          *out_cloud, "rgb");
    if (include_intensity)
      oint = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(
          *out_cloud, "intensity");
    for (size_t i = 0; i < obstacle_pts.size(); ++i, ++ox, ++oy, ++oz) {
      const auto &p = obstacle_pts[i];
      *ox = p.x;
      *oy = p.y;
      *oz = p.z;
      auto key = make_key(p.x, p.y, p.z);
      auto it = point_meta.find(key);
      if (it != point_meta.end()) {
        if (include_rgb && it->second.has_rgb && orgb) {
          *(*orgb) = it->second.rgb;
          ++(*orgb);
        }
        if (include_intensity && it->second.has_intensity && oint) {
          *(*oint) = it->second.intensity;
          ++(*oint);
        }
      } else {
        if (include_rgb && orgb) {
          *(*orgb) = 0.0f;
          ++(*orgb);
        }
        if (include_intensity && oint) {
          *(*oint) = 0.0f;
          ++(*oint);
        }
      }
    }
    pub_obstacle_cloud->publish(*out_cloud);
  }
}

} // namespace obstacle_detector_node_helpers
