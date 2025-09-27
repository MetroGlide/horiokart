// SPDX-License-Identifier: Apache-2.0
#ifndef HORIOKART_OBSTACLE_DETECTOR_3D_OBSTACLE_DETECTOR_NODE_HPP_
#define HORIOKART_OBSTACLE_DETECTOR_3D_OBSTACLE_DETECTOR_NODE_HPP_

#include <tf2/time.h>

#include <Eigen/Dense>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "horiokart_obstacle_detector_3d/core/cloud_processor.hpp"

// 公開ヘッダに重いヘッダを直接含めないため、tf2_ros 型を前方宣言します
namespace tf2_ros
{
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace obstacle_detector
{
class ScanProjector;
class ClusterDetector;
class GroundSeparator;
struct PointXYZ;
class GridHeightMap;
class CloudProcessor;
}  // namespace obstacle_detector

class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

  // コールバック
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  // セットアップ用ヘルパー
  void setupParameters();
  void setupCoreComponents();
  void setupPublishersAndSubscribers();

  // 主処理エントリ（cloudCallback から切り出したもの）
  void processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & msg);

  // ユーティリティ
  std::string make_key(double x, double y, double z) const;
  geometry_msgs::msg::TransformStamped getSensorTransform(const tf2::TimePoint & when);
  Eigen::Vector3d getSensorForward(const tf2::TimePoint & when);
  bool checkFootprintTraversable(const obstacle_detector::GridHeightMap & grid, double lookahead_m);

  // パラメータ（元実装の public ライクなメンバをここで管理）
  double publish_rate_;
  std::string input_topic_;
  std::string output_cloud_topic_;
  std::string output_scan_topic_;
  std::string diagnostics_topic_;
  double angle_min_;
  double angle_max_;
  double angle_inc_;
  double range_max_;
  double voxel_leaf_size_;
  double outlier_radius_;
  int outlier_min_neighbors_;
  double grid_cell_size_;
  double slope_threshold_deg_;
  double ground_max_distance_;
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  double min_obstacle_height_;
  double min_obstacle_volume_;
  double roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, roi_z_min_, roi_z_max_;
  bool use_rgb_;
  bool use_intensity_;
  bool publish_empty_scan_;

  int min_obs_per_cell_for_confident_median_;
  int radius_interp_cells_;
  double interp_power_p_;
  double interp_alpha_;
  double max_interp_area_m2_;
  double temporal_alpha_height_;
  double temporal_alpha_conf_;
  double observation_timeout_;

  double base_slope_threshold_deg_;
  double k_v_;
  std::string slope_method_;
  double pca_radius_m_;
  int pca_min_points_;

  bool intensity_compensate_distance_;
  double intensity_distance_ref_;
  double intensity_distance_power_;
  bool intensity_compensate_angle_;
  double intensity_angle_min_dot_;

  // パブリッシャ / サブスクライバ / core コンポーネント
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_confidence_cloud_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_traversable_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::shared_ptr<obstacle_detector::ScanProjector> projector_;
  std::shared_ptr<obstacle_detector::ClusterDetector> cluster_detector_;
  std::shared_ptr<obstacle_detector::GroundSeparator> ground_separator_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;
  std::string sensor_frame_;

  // フットプリント関連
  double footprint_width_;
  double footprint_lookahead_;
  double footprint_ground_fraction_;

  // TF キャッシュ
  std::optional<geometry_msgs::msg::TransformStamped> cached_sensor_tf_;
  rclcpp::Time cached_sensor_tf_time_;
  std::mutex tf_cache_mutex_;
  double tf_cache_timeout_sec_;

  bool enable_markers_;
  // 再配信時に元点の色/強度を参照するためのメタデータ
  using ColorInfo = obstacle_detector::ColorInfo;

  // processCloud から読みやすさ・テスト性のために切り出したヘルパメソッド
  sensor_msgs::msg::PointCloud2::SharedPtr transformCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg);
  void extractPointsAndMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf,
    std::vector<obstacle_detector::PointXYZ> & pts,
    std::unordered_map<std::string, ColorInfo> & point_meta, bool & has_rgb_field,
    bool & has_intensity_field);
  std::unique_ptr<obstacle_detector::GridHeightMap> buildGridFromPoints(
    const std::vector<obstacle_detector::PointXYZ> & pts);
  void publishConfidenceCloud(
    const obstacle_detector::GridHeightMap & grid,
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf);
  void publishObstacleCloudAndScan(
    const std::vector<obstacle_detector::PointXYZ> & obstacle_pts,
    const std::unordered_map<std::string, ColorInfo> & point_meta,
    const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf);

  // ノードラッパが使用するヘッドレスな core の CloudProcessor インスタンス
  std::unique_ptr<obstacle_detector::CloudProcessor> cloud_processor_;
};

#endif  // HORIOKART_OBSTACLE_DETECTOR_3D_OBSTACLE_DETECTOR_NODE_HPP_
