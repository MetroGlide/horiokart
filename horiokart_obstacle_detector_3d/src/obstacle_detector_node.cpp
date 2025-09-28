#include "horiokart_obstacle_detector_3d/obstacle_detector_node.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <algorithm>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <iomanip>
#include <limits>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"
#include "horiokart_obstacle_detector_3d/core/color_utils.hpp"
#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"
#include "horiokart_obstacle_detector_3d/core/intensity_utils.hpp"
#include "horiokart_obstacle_detector_3d/core/pca_slope_estimator.hpp"
#include "horiokart_obstacle_detector_3d/core/scan_projector.hpp"
#include "horiokart_obstacle_detector_3d/node_helpers/grid_utils.hpp"
#include "horiokart_obstacle_detector_3d/node_helpers/point_extractor.hpp"
#include "horiokart_obstacle_detector_3d/node_helpers/publish_utils.hpp"

// 実装: ヘルパー関数へ委譲してノードを薄く保ちます

ObstacleDetectorNode::ObstacleDetectorNode() : Node("obstacle_detector_node") {
  setupParameters();
  setupCoreComponents();
  setupPublishersAndSubscribers();
  // core の CloudProcessor を設定に従って生成
  obstacle_detector::CloudProcessorConfig cp_cfg;
  cp_cfg.roi_x_min = roi_x_min_;
  cp_cfg.roi_x_max = roi_x_max_;
  cp_cfg.roi_y_min = roi_y_min_;
  cp_cfg.roi_y_max = roi_y_max_;
  cp_cfg.grid_cell_size = grid_cell_size_;
  cp_cfg.slope_method = slope_method_;
  cp_cfg.pca_radius_m = pca_radius_m_;
  cp_cfg.pca_min_points = pca_min_points_;
  cp_cfg.voxel_leaf_size = voxel_leaf_size_;
  cp_cfg.dynamic_leaf = true;
  cp_cfg.target_points = 50000;
  cp_cfg.ground_max_distance = ground_max_distance_;
  cp_cfg.min_obstacle_height = min_obstacle_height_;
  cp_cfg.min_obstacle_volume = min_obstacle_volume_;
  cp_cfg.use_rgb = use_rgb_;
  cp_cfg.use_intensity = use_intensity_;
  cloud_processor_ =
      std::make_unique<obstacle_detector::CloudProcessor>(cp_cfg);
}

void ObstacleDetectorNode::setupParameters() {
  // パラメータを宣言（デフォルト値を設定）
  this->declare_parameter<std::string>("fixed_frame", "base_link");
  this->declare_parameter<std::string>("sensor_frame", "camera_link");
  this->declare_parameter<double>("processing_rate", 15.0);
  this->declare_parameter<std::string>("input_cloud_topic",
                                       "/camera/depth/points");
  this->declare_parameter<std::string>("output_cloud_topic",
                                       "/obstacle_points");
  this->declare_parameter<std::string>("output_scan_topic", "/obstacle_scan");
  this->declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
  this->declare_parameter<double>("scan_angle_min", -1.57);
  this->declare_parameter<double>("scan_angle_max", 1.57);
  this->declare_parameter<double>("scan_angle_increment", 0.01745);
  this->declare_parameter<double>("scan_range_max", 10.0);
  this->declare_parameter<double>("voxel_leaf_size", 0.03);
  this->declare_parameter<double>("outlier_radius", 0.05);
  this->declare_parameter<int>("outlier_min_neighbors", 2);
  this->declare_parameter<double>("grid_cell_size", 0.05);
  this->declare_parameter<int>("min_obs_per_cell_for_confident_median", 3);
  this->declare_parameter<int>("radius_interp_cells", 3);
  this->declare_parameter<double>("interp_power_p", 2.0);
  this->declare_parameter<double>("interp_alpha", 0.6);
  this->declare_parameter<double>("max_interp_area_m2", 0.5);
  this->declare_parameter<double>("temporal_alpha_height", 0.3);
  this->declare_parameter<double>("temporal_alpha_conf", 0.4);
  this->declare_parameter<double>("observation_timeout", 0.5);

  this->declare_parameter<double>("base_slope_threshold_deg", 15.0);
  this->declare_parameter<double>("k_v", 0.3);
  this->declare_parameter<double>("slope_threshold_deg", 15.0);
  this->declare_parameter<std::string>("slope_method", "finite_difference");
  this->declare_parameter<double>("pca_radius_m", 0.15);
  this->declare_parameter<int>("pca_min_points", 10);
  this->declare_parameter<double>("ground_max_distance", 0.08);
  this->declare_parameter<double>("cluster_tolerance", 0.1);
  this->declare_parameter<int>("min_cluster_size", 30);
  this->declare_parameter<int>("max_cluster_size", 1000000);
  this->declare_parameter<double>("min_obstacle_height", 0.08);
  this->declare_parameter<double>("min_obstacle_volume", 0.002);
  this->declare_parameter<bool>("use_color", false);
  this->declare_parameter<bool>("use_intensity", false);
  this->declare_parameter<bool>("publish_empty_scan", true);
  this->declare_parameter<bool>("enable_visualization_markers", true);
  this->declare_parameter<std::vector<double>>(
      "ground_score_weights", std::vector<double>{0.45, 0.35, 0.2});
  this->declare_parameter<double>("ground_ema_alpha", 0.3);
  this->declare_parameter<double>("ground_high_threshold", 0.7);
  this->declare_parameter<double>("ground_low_threshold", 0.4);
  this->declare_parameter<double>("footprint_width", 0.6);
  this->declare_parameter<double>("footprint_lookahead", 1.0);
  this->declare_parameter<double>("footprint_ground_fraction", 0.8);
  this->declare_parameter<double>("roi.x_min", 0.1);
  this->declare_parameter<double>("roi.x_max", 5.0);
  this->declare_parameter<double>("roi.y_min", -1.5);
  this->declare_parameter<double>("roi.y_max", 1.5);
  this->declare_parameter<double>("roi.z_min", -1.0);
  this->declare_parameter<double>("roi.z_max", 2.0);

  // パラメータ値をメンバへ読み込む
  target_frame_ = this->get_parameter("fixed_frame").as_string();
  sensor_frame_ = this->get_parameter("sensor_frame").as_string();
  publish_rate_ = this->get_parameter("processing_rate").as_double();
  input_topic_ = this->get_parameter("input_cloud_topic").as_string();
  output_cloud_topic_ = this->get_parameter("output_cloud_topic").as_string();
  output_scan_topic_ = this->get_parameter("output_scan_topic").as_string();
  diagnostics_topic_ = this->get_parameter("diagnostics_topic").as_string();
  angle_min_ = this->get_parameter("scan_angle_min").as_double();
  angle_max_ = this->get_parameter("scan_angle_max").as_double();
  angle_inc_ = this->get_parameter("scan_angle_increment").as_double();
  range_max_ = this->get_parameter("scan_range_max").as_double();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  outlier_radius_ = this->get_parameter("outlier_radius").as_double();
  outlier_min_neighbors_ =
      this->get_parameter("outlier_min_neighbors").as_int();
  grid_cell_size_ = this->get_parameter("grid_cell_size").as_double();
  min_obs_per_cell_for_confident_median_ =
      this->get_parameter("min_obs_per_cell_for_confident_median").as_int();
  radius_interp_cells_ = this->get_parameter("radius_interp_cells").as_int();
  interp_power_p_ = this->get_parameter("interp_power_p").as_double();
  interp_alpha_ = this->get_parameter("interp_alpha").as_double();
  max_interp_area_m2_ = this->get_parameter("max_interp_area_m2").as_double();
  temporal_alpha_height_ =
      this->get_parameter("temporal_alpha_height").as_double();
  temporal_alpha_conf_ = this->get_parameter("temporal_alpha_conf").as_double();
  observation_timeout_ = this->get_parameter("observation_timeout").as_double();

  base_slope_threshold_deg_ =
      this->get_parameter("base_slope_threshold_deg").as_double();
  k_v_ = this->get_parameter("k_v").as_double();
  slope_threshold_deg_ = this->get_parameter("slope_threshold_deg").as_double();
  ground_max_distance_ = this->get_parameter("ground_max_distance").as_double();
  cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
  min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
  max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
  min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
  min_obstacle_volume_ = this->get_parameter("min_obstacle_volume").as_double();
  use_rgb_ = this->get_parameter("use_color").as_bool();
  use_intensity_ = this->get_parameter("use_intensity").as_bool();
  publish_empty_scan_ = this->get_parameter("publish_empty_scan").as_bool();
  enable_markers_ =
      this->get_parameter("enable_visualization_markers").as_bool();
  std::vector<double> gsw = this->get_parameter_or<std::vector<double>>(
      "ground_score_weights", std::vector<double>{0.45, 0.35, 0.2});
  double footprint_w = this->get_parameter_or<double>("footprint_width", 0.6);
  double footprint_look =
      this->get_parameter_or<double>("footprint_lookahead", 1.0);
  double footprint_frac =
      this->get_parameter_or<double>("footprint_ground_fraction", 0.8);
  if (gsw.size() >= 3 && ground_separator_) {
    ground_separator_->setScoreWeights(gsw[0], gsw[1], gsw[2]);
  }
  // ground_separator のヒステリシスは setupCoreComponents で設定されます
  footprint_width_ = footprint_w;
  footprint_lookahead_ = footprint_look;
  footprint_ground_fraction_ = footprint_frac;
  slope_method_ = this->get_parameter_or<std::string>(
      "slope_method", std::string("finite_difference"));
  pca_radius_m_ = this->get_parameter_or<double>("pca_radius_m", 0.15);
  pca_min_points_ = this->get_parameter_or<int>("pca_min_points", 10);
  intensity_compensate_distance_ =
      this->get_parameter_or<bool>("intensity_compensate_distance", true);
  intensity_distance_ref_ =
      this->get_parameter_or<double>("intensity_distance_ref", 1.0);
  intensity_distance_power_ =
      this->get_parameter_or<double>("intensity_distance_power", 2.0);
  intensity_compensate_angle_ =
      this->get_parameter_or<bool>("intensity_compensate_angle", false);
  intensity_angle_min_dot_ =
      this->get_parameter_or<double>("intensity_angle_min_dot", 0.2);
  roi_x_min_ = this->get_parameter("roi.x_min").as_double();
  roi_x_max_ = this->get_parameter("roi.x_max").as_double();
  roi_y_min_ = this->get_parameter("roi.y_min").as_double();
  roi_y_max_ = this->get_parameter("roi.y_max").as_double();
  roi_z_min_ = this->get_parameter("roi.z_min").as_double();
  roi_z_max_ = this->get_parameter("roi.z_max").as_double();

  // TF キャッシュの初期値を設定
  tf_cache_timeout_sec_ = 1.0;
  cached_sensor_tf_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void ObstacleDetectorNode::setupCoreComponents() {
  // パブリッシャは setupPublishersAndSubscribers で作成しますが、
  // core コンポーネントはここで初期化します
  projector_ = std::make_shared<obstacle_detector::ScanProjector>(
      angle_min_, angle_max_, angle_inc_, range_max_);
  cluster_detector_ = std::make_shared<obstacle_detector::ClusterDetector>();
  cluster_detector_->setParameters(cluster_tolerance_, min_cluster_size_,
                                   max_cluster_size_);
  cluster_detector_->setDownsampleLeafSize(voxel_leaf_size_);
  cluster_detector_->setOutlierRadius(outlier_radius_);
  cluster_detector_->setOutlierMinNeighbors(outlier_min_neighbors_);

  ground_separator_ = std::make_shared<obstacle_detector::GroundSeparator>();
  ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
  // パラメータからヒステリシスを設定
  double g_alpha = this->get_parameter_or<double>("ground_ema_alpha", 0.3);
  double g_high = this->get_parameter_or<double>("ground_high_threshold", 0.7);
  double g_low = this->get_parameter_or<double>("ground_low_threshold", 0.4);
  ground_separator_->setHysteresisParameters(g_alpha, g_high, g_low);

  // TF バッファとリスナの初期化
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetectorNode::setupPublishersAndSubscribers() {
  pub_obstacle_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud_topic_, 10);
  pub_confidence_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/confidence_map", 1);
  pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_scan_topic_, 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/obstacle_markers", 1);
  pub_traversable_ =
      this->create_publisher<std_msgs::msg::Bool>("/footprint_traversable", 1);
  pub_diagnostics_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          diagnostics_topic_, 1);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::odomCallback, this,
                std::placeholders::_1));
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleDetectorNode::cloudCallback, this,
                std::placeholders::_1));

  // 動的パラメータ用コールバック
  param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "";
        bool scan_params_changed = false;
        for (const auto &p : params) {
          const std::string &name = p.get_name();
          if (name == "voxel_leaf_size") {
            voxel_leaf_size_ = p.as_double();
            if (cluster_detector_)
              cluster_detector_->setDownsampleLeafSize(voxel_leaf_size_);
          } else if (name == "outlier_radius") {
            outlier_radius_ = p.as_double();
            if (cluster_detector_)
              cluster_detector_->setOutlierRadius(outlier_radius_);
          } else if (name == "outlier_min_neighbors") {
            outlier_min_neighbors_ = p.as_int();
            if (cluster_detector_)
              cluster_detector_->setOutlierMinNeighbors(outlier_min_neighbors_);
          } else if (name == "cluster_tolerance") {
            cluster_tolerance_ = p.as_double();
            if (cluster_detector_)
              cluster_detector_->setParameters(
                  cluster_tolerance_, min_cluster_size_, max_cluster_size_);
          } else if (name == "min_cluster_size") {
            min_cluster_size_ = p.as_int();
            if (cluster_detector_)
              cluster_detector_->setParameters(
                  cluster_tolerance_, min_cluster_size_, max_cluster_size_);
          } else if (name == "max_cluster_size") {
            max_cluster_size_ = p.as_int();
            if (cluster_detector_)
              cluster_detector_->setParameters(
                  cluster_tolerance_, min_cluster_size_, max_cluster_size_);
          } else if (name == "slope_threshold_deg") {
            slope_threshold_deg_ = p.as_double();
            if (ground_separator_)
              ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
          } else if (name == "fixed_frame") {
            target_frame_ = p.as_string();
          } else if (name == "sensor_frame") {
            sensor_frame_ = p.as_string();
          } else if (name == "ground_max_distance") {
            ground_max_distance_ = p.as_double();
          } else if (name == "use_color") {
            use_rgb_ = p.as_bool();
          } else if (name == "use_intensity") {
            use_intensity_ = p.as_bool();
          } else if (name == "publish_empty_scan") {
            publish_empty_scan_ = p.as_bool();
          } else if (name == "scan_angle_min") {
            angle_min_ = p.as_double();
            scan_params_changed = true;
          } else if (name == "scan_angle_max") {
            angle_max_ = p.as_double();
            scan_params_changed = true;
          } else if (name == "scan_angle_increment") {
            angle_inc_ = p.as_double();
            scan_params_changed = true;
          } else if (name == "scan_range_max") {
            range_max_ = p.as_double();
            scan_params_changed = true;
          }
        }
        if (scan_params_changed) {
          projector_ = std::make_shared<obstacle_detector::ScanProjector>(
              angle_min_, angle_max_, angle_inc_, range_max_);
        }
        return result;
      });
}

void ObstacleDetectorNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double v = std::hypot(vx, vy);
  double factor = std::max(0.5, 1.0 - k_v_ * v);
  slope_threshold_deg_ = base_slope_threshold_deg_ * factor;
  if (ground_separator_) {
    ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
  }
}

// ユーティリティ: 座標を文字列キーへ変換（丸め込み）
std::string ObstacleDetectorNode::make_key(double x, double y, double z) const {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4) << x << "," << y << "," << z;
  return ss.str();
}

// TF 関連とフットプリント判定のユーティリティ実装
geometry_msgs::msg::TransformStamped
ObstacleDetectorNode::getSensorTransform(const tf2::TimePoint &when) {
  geometry_msgs::msg::TransformStamped tfst;
  if (!tf_buffer_) {
    throw tf2::TransformException("No TF buffer");
  }
  try {
    tfst = tf_buffer_->lookupTransform(target_frame_, sensor_frame_, when);
    std::lock_guard<std::mutex> lk(tf_cache_mutex_);
    cached_sensor_tf_ = tfst;
    cached_sensor_tf_time_ = this->now();
    return tfst;
  } catch (const tf2::TransformException &ex) {
    std::lock_guard<std::mutex> lk(tf_cache_mutex_);
    if (cached_sensor_tf_) {
      double age = (this->now() - cached_sensor_tf_time_).seconds();
      if (age <= tf_cache_timeout_sec_) {
        return *cached_sensor_tf_;
      }
    }
    throw;
  }
}

Eigen::Vector3d
ObstacleDetectorNode::getSensorForward(const tf2::TimePoint &when) {
  Eigen::Vector3d sensor_fwd(1.0, 0.0, 0.0);
  try {
    auto tfst = getSensorTransform(when);
    const auto &r = tfst.transform.rotation;
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    sensor_fwd = q * Eigen::Vector3d(1.0, 0.0, 0.0);
    sensor_fwd.normalize();
  } catch (const tf2::TransformException &ex) {
    RCLCPP_DEBUG(this->get_logger(),
                 "getSensorForward: using fallback +x due to TF error: %s",
                 ex.what());
  }
  return sensor_fwd;
}

bool ObstacleDetectorNode::checkFootprintTraversable(
    const obstacle_detector::GridHeightMap &grid, double lookahead_m) {
  int ix0 = static_cast<int>(std::floor((0.0 - roi_x_min_) / grid_cell_size_));
  int ix1 = static_cast<int>(
      std::floor((lookahead_m - roi_x_min_) / grid_cell_size_));
  int half_w_cells =
      static_cast<int>(std::ceil((footprint_width_ / 2.0) / grid_cell_size_));
  int iy_center =
      static_cast<int>(std::floor((0.0 - roi_y_min_) / grid_cell_size_));
  int required = 0;
  int total = 0;
  for (int ix = ix0; ix <= ix1; ++ix) {
    for (int dy = -half_w_cells; dy <= half_w_cells; ++dy) {
      int iy = iy_center + dy;
      obstacle_detector::GridCell cell;
      if (!grid.getCell(ix, iy, cell) || !cell.has_observation) {
        continue;
      }
      total++;
      if (cell.confidence >= 0.5 &&
          cell.ground_ema >= ground_separator_->getHighThreshold()) {
        required++;
      }
    }
  }

  if (total == 0) {
    return false;
  }
  double frac = static_cast<double>(required) / static_cast<double>(total);
  return frac >= footprint_ground_fraction_;
}

// PointCloud サブスクライブコールバックは processCloud へ委譲
void ObstacleDetectorNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  processCloud(msg);
}

// processCloud: 点群処理の主エントリ
void ObstacleDetectorNode::processCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  const auto t0 = std::chrono::high_resolution_clock::now();

  // 1) 入力点群をターゲットフレームへ変換
  auto cloud_in_tf = transformCloud(msg);

  // 2) 点抽出とメタデータ取得（node_helpers に委譲）
  std::vector<obstacle_detector::PointXYZ> pts;
  std::unordered_map<std::string, ColorInfo> point_meta;
  bool has_rgb_field = false;
  bool has_intensity_field = false;
  std::unordered_map<std::string, obstacle_detector::PointXYZ> dummy_meta;
  obstacle_detector_node_helpers::extractPointsAndMeta(
      cloud_in_tf, pts, dummy_meta, point_meta, has_rgb_field,
      has_intensity_field, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_,
      roi_z_min_, roi_z_max_, use_rgb_, use_intensity_, intensity_distance_ref_,
      intensity_distance_power_, intensity_compensate_distance_,
      intensity_compensate_angle_, intensity_angle_min_dot_,
      [this](const tf2::TimePoint &t) { return this->getSensorForward(t); },
      [this](const tf2::TimePoint &t) { return this->getSensorTransform(t); },
      [this](double x, double y, double z) { return this->make_key(x, y, z); },
      [this](const std::string &k, const std::vector<double> &d) {
        return this->get_parameter_or<std::vector<double>>(k, d);
      });
  if (pts.empty())
    return;

  // 3) core (CloudProcessor) で処理
  if (!cloud_processor_)
    return;
  auto res = cloud_processor_->process(pts, point_meta, *cluster_detector_,
                                       *ground_separator_);

  // 4) フットプリントの走行可能判定を publish（ノード固有ルール）
  bool traversable = checkFootprintTraversable(*res.grid, footprint_lookahead_);
  if (pub_traversable_ && pub_traversable_->get_subscription_count() > 0) {
    std_msgs::msg::Bool m;
    m.data = traversable;
    pub_traversable_->publish(m);
  }

  // 5) 信頼度グリッドを publish（node_helpers に委譲）
  obstacle_detector_node_helpers::publishConfidenceCloud(
      *res.grid, cloud_in_tf, pub_confidence_cloud_, roi_x_min_, roi_y_min_,
      grid_cell_size_);

  // 6) 障害物点群とスキャンを publish（node_helpers に委譲）
  obstacle_detector_node_helpers::publishObstacleCloudAndScan(
      res.obstacle_points, point_meta, cloud_in_tf, pub_obstacle_cloud_,
      pub_scan_, use_rgb_, use_intensity_,
      [this](double x, double y, double z) { return this->make_key(x, y, z); },
      [this](const std::string &k, const std::vector<double> &d) {
        return this->get_parameter_or<std::vector<double>>(k, d);
      });

  // 診断情報の publish
  if (pub_diagnostics_ && pub_diagnostics_->get_subscription_count() > 0) {
    const auto t1 = std::chrono::high_resolution_clock::now();
    double proc_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
            t1 - t0)
            .count();
    diagnostic_msgs::msg::DiagnosticArray darr;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name = "horiokart_obstacle_detector_3d";
    status.message = "Processing status";
    status.values.reserve(4);
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "processing_time_ms";
    kv.value = std::to_string(proc_ms);
    status.values.push_back(kv);
    kv.key = "input_points";
    kv.value = std::to_string(pts.size());
    status.values.push_back(kv);
    kv.key = "obstacle_points";
    kv.value = std::to_string(res.obstacle_points.size());
    status.values.push_back(kv);
    kv.key = "publish_rate_configured";
    kv.value = std::to_string(publish_rate_);
    status.values.push_back(kv);
    darr.status.push_back(status);
    darr.header.stamp = now();
    pub_diagnostics_->publish(darr);
  }

  // 可視化用マーカーの生成と publish
  if (enable_markers_ && pub_markers_ &&
      pub_markers_->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker grid_marker;
    grid_marker.header = cloud_in_tf->header;
    grid_marker.ns = "grid_conf";
    grid_marker.id = 0;
    grid_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    grid_marker.action = visualization_msgs::msg::Marker::ADD;
    grid_marker.scale.x = static_cast<float>(grid_cell_size_);
    grid_marker.scale.y = static_cast<float>(grid_cell_size_);
    grid_marker.scale.z = 0.02f;
    for (int ix = 0; ix < res.grid->rows(); ++ix) {
      for (int iy = 0; iy < res.grid->cols(); ++iy) {
        obstacle_detector::GridCell c;
        if (!res.grid->getCell(ix, iy, c) || !c.has_observation)
          continue;
        geometry_msgs::msg::Point pt;
        double cx = roi_x_min_ + (ix + 0.5) * grid_cell_size_;
        double cy = roi_y_min_ + (iy + 0.5) * grid_cell_size_;
        pt.x = static_cast<float>(cx);
        pt.y = static_cast<float>(cy);
        pt.z = static_cast<float>(c.height_median + 0.01);
        grid_marker.points.push_back(pt);
        std_msgs::msg::ColorRGBA col;
        col.a = 0.8f;
        col.r = static_cast<float>(1.0 - c.confidence);
        col.g = static_cast<float>(c.confidence);
        col.b = 0.0f;
        grid_marker.colors.push_back(col);
      }
    }
    ma.markers.push_back(grid_marker);
    int mid = 1;
    for (const auto &c : res.clusters) {
      double xmin = 1e9, ymin = 1e9, zmin = 1e9, xmax = -1e9, ymax = -1e9,
             zmax = -1e9;
      for (const auto &p : c.points) {
        xmin = std::min(xmin, static_cast<double>(p.x));
        ymin = std::min(ymin, static_cast<double>(p.y));
        zmin = std::min(zmin, static_cast<double>(p.z));
        xmax = std::max(xmax, static_cast<double>(p.x));
        ymax = std::max(ymax, static_cast<double>(p.y));
        zmax = std::max(zmax, static_cast<double>(p.z));
      }
      visualization_msgs::msg::Marker m;
      m.header = cloud_in_tf->header;
      m.ns = "clusters";
      m.id = mid++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = static_cast<float>((xmin + xmax) * 0.5);
      m.pose.position.y = static_cast<float>((ymin + ymax) * 0.5);
      m.pose.position.z = static_cast<float>((zmin + zmax) * 0.5);
      m.scale.x = static_cast<float>(xmax - xmin);
      m.scale.y = static_cast<float>(ymax - ymin);
      m.scale.z = static_cast<float>(zmax - zmin);
      m.color.a = 0.6f;
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      ma.markers.push_back(m);
    }
    pub_markers_->publish(ma);
  }
}

// ヘルパー実装
sensor_msgs::msg::PointCloud2::SharedPtr ObstacleDetectorNode::transformCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_in_tf = msg;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != target_frame_) {
    try {
      geometry_msgs::msg::TransformStamped tfst = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id,
          tf2::TimePoint(std::chrono::seconds(msg->header.stamp.sec) +
                         std::chrono::nanoseconds(msg->header.stamp.nanosec)));
      auto transformed = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*msg, *transformed, tfst);
      transformed->header.frame_id = target_frame_;
      cloud_in_tf = transformed;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "TF transform failed: %s. Processing in original frame.",
                  ex.what());
    }
  }
  return cloud_in_tf;
}

void ObstacleDetectorNode::extractPointsAndMeta(
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf,
    std::vector<obstacle_detector::PointXYZ> &pts,
    std::unordered_map<std::string, ColorInfo> &point_meta, bool &has_rgb_field,
    bool &has_intensity_field) {
  // node_helpers の実装に委譲（ノードを薄く保つ）
  std::unordered_map<std::string, obstacle_detector::PointXYZ> dummy_meta;
  obstacle_detector_node_helpers::extractPointsAndMeta(
      cloud_in_tf, pts, dummy_meta, point_meta, has_rgb_field,
      has_intensity_field, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_,
      roi_z_min_, roi_z_max_, use_rgb_, use_intensity_, intensity_distance_ref_,
      intensity_distance_power_, intensity_compensate_distance_,
      intensity_compensate_angle_, intensity_angle_min_dot_,
      [this](const tf2::TimePoint &t) { return this->getSensorForward(t); },
      [this](const tf2::TimePoint &t) { return this->getSensorTransform(t); },
      [this](double x, double y, double z) { return this->make_key(x, y, z); },
      [this](const std::string &k, const std::vector<double> &d) {
        return this->get_parameter_or<std::vector<double>>(k, d);
      });
}

std::unique_ptr<obstacle_detector::GridHeightMap>
ObstacleDetectorNode::buildGridFromPoints(
    const std::vector<obstacle_detector::PointXYZ> &pts) {
  return obstacle_detector_node_helpers::buildGridFromPoints(
      pts, roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, grid_cell_size_,
      min_obs_per_cell_for_confident_median_, radius_interp_cells_,
      interp_power_p_, interp_alpha_, max_interp_area_m2_,
      temporal_alpha_height_, temporal_alpha_conf_, observation_timeout_);
}

// ノードは CloudProcessor に処理を委譲します

void ObstacleDetectorNode::publishConfidenceCloud(
    const obstacle_detector::GridHeightMap &grid,
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf) {
  obstacle_detector_node_helpers::publishConfidenceCloud(
      grid, cloud_in_tf, pub_confidence_cloud_, roi_x_min_, roi_y_min_,
      grid_cell_size_);
}

void ObstacleDetectorNode::publishObstacleCloudAndScan(
    const std::vector<obstacle_detector::PointXYZ> &obstacle_pts,
    const std::unordered_map<std::string, ColorInfo> &point_meta,
    const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_in_tf) {
  obstacle_detector_node_helpers::publishObstacleCloudAndScan(
      obstacle_pts, point_meta, cloud_in_tf, pub_obstacle_cloud_, pub_scan_,
      use_rgb_, use_intensity_,
      [this](double x, double y, double z) { return this->make_key(x, y, z); },
      [this](const std::string &k, const std::vector<double> &d) {
        return this->get_parameter_or<std::vector<double>>(k, d);
      });
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
