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
#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"
#include "horiokart_obstacle_detector_3d/core/ground_separator.hpp"
#include "horiokart_obstacle_detector_3d/core/intensity_utils.hpp"
#include "horiokart_obstacle_detector_3d/core/pca_slope_estimator.hpp"
#include "horiokart_obstacle_detector_3d/core/scan_projector.hpp"

// helper: unpack packed float RGB (IEEE 754) into r,g,b 0..1
static void unpackFloatRGB(float rgb_float, double & r, double & g, double & b)
{
  uint32_t u = *reinterpret_cast<uint32_t *>(&rgb_float);
  uint8_t ri = (u >> 16) & 0xFF;
  uint8_t gi = (u >> 8) & 0xFF;
  uint8_t bi = u & 0xFF;
  r = ri / 255.0;
  g = gi / 255.0;
  b = bi / 255.0;
}

// rgb 0..1 -> hsv (h:0..360, s:0..1, v:0..1)
static void rgbToHsv(double r, double g, double b, double & h, double & s, double & v)
{
  double mx = std::max(r, std::max(g, b));
  double mn = std::min(r, std::min(g, b));
  v = mx;
  double d = mx - mn;
  s = (mx == 0.0) ? 0.0 : d / mx;
  if (d == 0.0) {
    h = 0.0;
    return;
  }
  if (mx == r) {
    h = 60.0 * (fmod(((g - b) / d), 6.0));
  } else if (mx == g) {
    h = 60.0 * (((b - r) / d) + 2.0);
  } else {
    h = 60.0 * (((r - g) / d) + 4.0);
  }
  if (h < 0.0) {
    h += 360.0;
  }
}

// Implementation now delegates to helper methods declared in header

ObstacleDetectorNode::ObstacleDetectorNode() : Node("obstacle_detector_node")
{
  setupParameters();
  setupCoreComponents();
  setupPublishersAndSubscribers();
}

void ObstacleDetectorNode::setupParameters()
{
  // declare parameters with defaults (kept consistent with previous file)
  this->declare_parameter<std::string>("fixed_frame", "base_link");
  this->declare_parameter<std::string>("sensor_frame", "camera_link");
  this->declare_parameter<double>("processing_rate", 15.0);
  this->declare_parameter<std::string>("input_cloud_topic", "/camera/depth/points");
  this->declare_parameter<std::string>("output_cloud_topic", "/obstacle_points");
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

  // read values into members
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
  outlier_min_neighbors_ = this->get_parameter("outlier_min_neighbors").as_int();
  grid_cell_size_ = this->get_parameter("grid_cell_size").as_double();
  min_obs_per_cell_for_confident_median_ =
    this->get_parameter("min_obs_per_cell_for_confident_median").as_int();
  radius_interp_cells_ = this->get_parameter("radius_interp_cells").as_int();
  interp_power_p_ = this->get_parameter("interp_power_p").as_double();
  interp_alpha_ = this->get_parameter("interp_alpha").as_double();
  max_interp_area_m2_ = this->get_parameter("max_interp_area_m2").as_double();
  temporal_alpha_height_ = this->get_parameter("temporal_alpha_height").as_double();
  temporal_alpha_conf_ = this->get_parameter("temporal_alpha_conf").as_double();
  observation_timeout_ = this->get_parameter("observation_timeout").as_double();

  base_slope_threshold_deg_ = this->get_parameter("base_slope_threshold_deg").as_double();
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
  enable_markers_ = this->get_parameter("enable_visualization_markers").as_bool();
  std::vector<double> gsw = this->get_parameter_or<std::vector<double>>(
    "ground_score_weights", std::vector<double>{0.45, 0.35, 0.2});
  double footprint_w = this->get_parameter_or<double>("footprint_width", 0.6);
  double footprint_look = this->get_parameter_or<double>("footprint_lookahead", 1.0);
  double footprint_frac = this->get_parameter_or<double>("footprint_ground_fraction", 0.8);
  if (gsw.size() >= 3 && ground_separator_) {
    ground_separator_->setScoreWeights(gsw[0], gsw[1], gsw[2]);
  }
  // ground separator hysteresis will be set in setupCoreComponents when object
  // exists
  footprint_width_ = footprint_w;
  footprint_lookahead_ = footprint_look;
  footprint_ground_fraction_ = footprint_frac;
  slope_method_ =
    this->get_parameter_or<std::string>("slope_method", std::string("finite_difference"));
  pca_radius_m_ = this->get_parameter_or<double>("pca_radius_m", 0.15);
  pca_min_points_ = this->get_parameter_or<int>("pca_min_points", 10);
  intensity_compensate_distance_ =
    this->get_parameter_or<bool>("intensity_compensate_distance", true);
  intensity_distance_ref_ = this->get_parameter_or<double>("intensity_distance_ref", 1.0);
  intensity_distance_power_ = this->get_parameter_or<double>("intensity_distance_power", 2.0);
  intensity_compensate_angle_ = this->get_parameter_or<bool>("intensity_compensate_angle", false);
  intensity_angle_min_dot_ = this->get_parameter_or<double>("intensity_angle_min_dot", 0.2);
  roi_x_min_ = this->get_parameter("roi.x_min").as_double();
  roi_x_max_ = this->get_parameter("roi.x_max").as_double();
  roi_y_min_ = this->get_parameter("roi.y_min").as_double();
  roi_y_max_ = this->get_parameter("roi.y_max").as_double();
  roi_z_min_ = this->get_parameter("roi.z_min").as_double();
  roi_z_max_ = this->get_parameter("roi.z_max").as_double();

  // initialize TF cache defaults
  tf_cache_timeout_sec_ = 1.0;
  cached_sensor_tf_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

void ObstacleDetectorNode::setupCoreComponents()
{
  // publishers will be created in setupPublishersAndSubscribers but core
  // instances here
  projector_ = std::make_shared<obstacle_detector::ScanProjector>(
    angle_min_, angle_max_, angle_inc_, range_max_);
  cluster_detector_ = std::make_shared<obstacle_detector::ClusterDetector>();
  cluster_detector_->setParameters(cluster_tolerance_, min_cluster_size_, max_cluster_size_);
  cluster_detector_->setDownsampleLeafSize(voxel_leaf_size_);
  cluster_detector_->setOutlierRadius(outlier_radius_);
  cluster_detector_->setOutlierMinNeighbors(outlier_min_neighbors_);

  ground_separator_ = std::make_shared<obstacle_detector::GroundSeparator>();
  ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
  // Set hysteresis from parameters
  double g_alpha = this->get_parameter_or<double>("ground_ema_alpha", 0.3);
  double g_high = this->get_parameter_or<double>("ground_high_threshold", 0.7);
  double g_low = this->get_parameter_or<double>("ground_low_threshold", 0.4);
  ground_separator_->setHysteresisParameters(g_alpha, g_high, g_low);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ObstacleDetectorNode::setupPublishersAndSubscribers()
{
  pub_obstacle_cloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, 10);
  pub_confidence_cloud_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("/confidence_map", 1);
  pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(output_scan_topic_, 10);
  pub_markers_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 1);
  pub_traversable_ = this->create_publisher<std_msgs::msg::Bool>("/footprint_traversable", 1);
  pub_diagnostics_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 1);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::odomCallback, this, std::placeholders::_1));
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleDetectorNode::cloudCallback, this, std::placeholders::_1));

  // dynamic param callback
  param_cb_handle_ =
    this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "";
      bool scan_params_changed = false;
      for (const auto & p : params) {
        const std::string & name = p.get_name();
        if (name == "voxel_leaf_size") {
          voxel_leaf_size_ = p.as_double();
          if (cluster_detector_) cluster_detector_->setDownsampleLeafSize(voxel_leaf_size_);
        } else if (name == "outlier_radius") {
          outlier_radius_ = p.as_double();
          if (cluster_detector_) cluster_detector_->setOutlierRadius(outlier_radius_);
        } else if (name == "outlier_min_neighbors") {
          outlier_min_neighbors_ = p.as_int();
          if (cluster_detector_) cluster_detector_->setOutlierMinNeighbors(outlier_min_neighbors_);
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
          if (ground_separator_) ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
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

void ObstacleDetectorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  double v = std::hypot(vx, vy);
  double factor = std::max(0.5, 1.0 - k_v_ * v);
  slope_threshold_deg_ = base_slope_threshold_deg_ * factor;
  if (ground_separator_) {
    ground_separator_->setSlopeThresholdDeg(slope_threshold_deg_);
  }
}

// utility make_key (stringified rounded coords)
std::string ObstacleDetectorNode::make_key(double x, double y, double z) const
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4) << x << "," << y << "," << z;
  return ss.str();
}

// keep original implementations of
// getSensorTransform/getSensorForward/checkFootprintTraversable
geometry_msgs::msg::TransformStamped ObstacleDetectorNode::getSensorTransform(
  const tf2::TimePoint & when)
{
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
  } catch (const tf2::TransformException & ex) {
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

Eigen::Vector3d ObstacleDetectorNode::getSensorForward(const tf2::TimePoint & when)
{
  Eigen::Vector3d sensor_fwd(1.0, 0.0, 0.0);
  try {
    auto tfst = getSensorTransform(when);
    const auto & r = tfst.transform.rotation;
    Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
    sensor_fwd = q * Eigen::Vector3d(1.0, 0.0, 0.0);
    sensor_fwd.normalize();
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(), "getSensorForward: using fallback +x due to TF error: %s", ex.what());
  }
  return sensor_fwd;
}

bool ObstacleDetectorNode::checkFootprintTraversable(
  const obstacle_detector::GridHeightMap & grid, double lookahead_m)
{
  int ix0 = static_cast<int>(std::floor((0.0 - roi_x_min_) / grid_cell_size_));
  int ix1 = static_cast<int>(std::floor((lookahead_m - roi_x_min_) / grid_cell_size_));
  int half_w_cells = static_cast<int>(std::ceil((footprint_width_ / 2.0) / grid_cell_size_));
  int iy_center = static_cast<int>(std::floor((0.0 - roi_y_min_) / grid_cell_size_));
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
      if (cell.confidence >= 0.5 && cell.ground_ema >= ground_separator_->getHighThreshold()) {
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

// Cloud callback forwards to processCloud to keep subscriber thin
void ObstacleDetectorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  processCloud(msg);
}

// processCloud: implementation follows (moved from the old free-function body
// below)

// Move the original cloud processing body into the class method processCloud.
void ObstacleDetectorNode::processCloud(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  const auto t0 = std::chrono::high_resolution_clock::now();

  // 1) transform cloud into target frame if needed
  auto cloud_in_tf = transformCloud(msg);

  // 2) extract points and metadata (rgb/intensity)
  std::vector<obstacle_detector::PointXYZ> pts;
  std::unordered_map<std::string, ColorInfo> point_meta;
  bool has_rgb_field = false;
  bool has_intensity_field = false;
  extractPointsAndMeta(cloud_in_tf, pts, point_meta, has_rgb_field, has_intensity_field);
  if (pts.empty()) {
    return;
  }

  // 3) build grid and perform pre-cluster processing
  auto grid = buildGridFromPoints(pts);

  // adjust downsample leaf size based on point count (same heuristic)
  bool dynamic_leaf = this->get_parameter_or<bool>("dynamic_leaf_size", true);
  int target_pts = this->get_parameter_or<int>("target_points", 50000);
  if (dynamic_leaf && pts.size() > 0) {
    double scale = std::sqrt(static_cast<double>(pts.size()) / static_cast<double>(target_pts));
    double new_leaf = voxel_leaf_size_ * scale;
    new_leaf = std::max(0.005, std::min(0.2, new_leaf));
    cluster_detector_->setDownsampleLeafSize(new_leaf);
  }

  // 4) classify points into ground / non-ground (may use PCA)
  std::unordered_map<int, std::pair<double, Eigen::Vector3d>> pca_results;
  if (slope_method_ == "pca") {
    pca_results = obstacle_detector::computePcaSlopesAndNormals(
      pts, *grid, pca_radius_m_, pca_min_points_, grid_cell_size_, roi_x_min_, roi_y_min_);
  }
  std::vector<obstacle_detector::PointXYZ> ground_pts, non_ground_pts;
  // create a local reference for easier access to grid methods/fields
  auto & grid_ref = *grid;
  classifyPoints(pts, point_meta, grid_ref, ground_pts, non_ground_pts, pca_results);

  // 5) publish traversability
  bool traversable = checkFootprintTraversable(grid_ref, footprint_lookahead_);
  if (pub_traversable_ && pub_traversable_->get_subscription_count() > 0) {
    std_msgs::msg::Bool m;
    m.data = traversable;
    pub_traversable_->publish(m);
  }

  // 6) publish confidence cloud if requested
  publishConfidenceCloud(grid_ref, cloud_in_tf);

  // 7) cluster non-ground points and filter by height/volume
  auto clusters = cluster_detector_->extractClusters(non_ground_pts);
  std::vector<obstacle_detector::PointXYZ> obstacle_pts;
  for (const auto & c : clusters) {
    double zmin = std::numeric_limits<double>::max(), zmax = -std::numeric_limits<double>::max();
    for (const auto & p : c.points) {
      zmin = std::min(zmin, (double)p.z);
      zmax = std::max(zmax, (double)p.z);
    }
    double height = zmax - zmin;
    if (height < min_obstacle_height_) continue;
    if (c.volume < min_obstacle_volume_) continue;
    obstacle_pts.insert(obstacle_pts.end(), c.points.begin(), c.points.end());
  }

  // 8) publish obstacle cloud and scan
  publishObstacleCloudAndScan(obstacle_pts, point_meta, cloud_in_tf);

  // diagnostics
  if (pub_diagnostics_ && pub_diagnostics_->get_subscription_count() > 0) {
    const auto t1 = std::chrono::high_resolution_clock::now();
    double proc_ms =
      std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t1 - t0).count();
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
    kv.value = std::to_string(obstacle_pts.size());
    status.values.push_back(kv);
    kv.key = "publish_rate_configured";
    kv.value = std::to_string(publish_rate_);
    status.values.push_back(kv);
    darr.status.push_back(status);
    darr.header.stamp = this->now();
    pub_diagnostics_->publish(darr);
  }

  // markers
  if (enable_markers_ && pub_markers_ && pub_markers_->get_subscription_count() > 0) {
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
    for (int ix = 0; ix < grid_ref.rows(); ++ix) {
      for (int iy = 0; iy < grid_ref.cols(); ++iy) {
        obstacle_detector::GridCell c;
        if (!grid_ref.getCell(ix, iy, c) || !c.has_observation) continue;
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
    for (const auto & c : clusters) {
      double xmin = 1e9, ymin = 1e9, zmin = 1e9, xmax = -1e9, ymax = -1e9, zmax = -1e9;
      for (const auto & p : c.points) {
        xmin = std::min(xmin, (double)p.x);
        ymin = std::min(ymin, (double)p.y);
        zmin = std::min(zmin, (double)p.z);
        xmax = std::max(xmax, (double)p.x);
        ymax = std::max(ymax, (double)p.y);
        zmax = std::max(zmax, (double)p.z);
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

// Helper implementations
sensor_msgs::msg::PointCloud2::SharedPtr ObstacleDetectorNode::transformCloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
{
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_in_tf = msg;
  if (!msg->header.frame_id.empty() && msg->header.frame_id != target_frame_) {
    try {
      geometry_msgs::msg::TransformStamped tfst = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id,
        tf2::TimePoint(
          std::chrono::seconds(msg->header.stamp.sec) +
          std::chrono::nanoseconds(msg->header.stamp.nanosec)));
      auto transformed = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*msg, *transformed, tfst);
      transformed->header.frame_id = target_frame_;
      cloud_in_tf = transformed;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "TF transform failed: %s. Processing in original frame.", ex.what());
    }
  }
  return cloud_in_tf;
}

void ObstacleDetectorNode::extractPointsAndMeta(
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf,
  std::vector<obstacle_detector::PointXYZ> & pts,
  std::unordered_map<std::string, ColorInfo> & point_meta, bool & has_rgb_field,
  bool & has_intensity_field)
{
  size_t approx =
    static_cast<size_t>(cloud_in_tf->width) * static_cast<size_t>(cloud_in_tf->height);
  pts.reserve(approx > 0 ? approx : 1024);
  has_rgb_field = false;
  has_intensity_field = false;
  for (const auto & f : cloud_in_tf->fields) {
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
    it_rgb = std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*cloud_in_tf, "rgb");
  if (has_intensity_field)
    it_intensity =
      std::make_unique<sensor_msgs::PointCloud2ConstIterator<float>>(*cloud_in_tf, "intensity");

  for (; itx != itx.end(); ++itx, ++ity, ++itz) {
    obstacle_detector::PointXYZ p{*itx, *ity, *itz};
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      if (has_rgb_field) ++(*it_rgb);
      if (has_intensity_field) ++(*it_intensity);
      continue;
    }
    if (
      p.x < roi_x_min_ || p.x > roi_x_max_ || p.y < roi_y_min_ || p.y > roi_y_max_ ||
      p.z < roi_z_min_ || p.z > roi_z_max_) {
      if (has_rgb_field) ++(*it_rgb);
      if (has_intensity_field) ++(*it_intensity);
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
    std::vector<double> inorm = this->get_parameter_or<std::vector<double>>(
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
      in = obstacle_detector::compensateIntensity(
        in, p, sensor_origin, sensor_fwd, intensity_distance_ref_, intensity_distance_power_,
        intensity_compensate_distance_, intensity_compensate_angle_, intensity_angle_min_dot_);
      ci.intensity = static_cast<float>(in);
    }
    point_meta.emplace(make_key(p.x, p.y, p.z), ci);
    if (has_rgb_field) ++(*it_rgb);
    if (has_intensity_field) ++(*it_intensity);
    pts.push_back(p);
  }
}

std::unique_ptr<obstacle_detector::GridHeightMap> ObstacleDetectorNode::buildGridFromPoints(
  const std::vector<obstacle_detector::PointXYZ> & pts)
{
  auto grid = std::make_unique<obstacle_detector::GridHeightMap>(
    roi_x_min_, roi_x_max_, roi_y_min_, roi_y_max_, grid_cell_size_);
  grid->setParameters(
    min_obs_per_cell_for_confident_median_, radius_interp_cells_, interp_power_p_, interp_alpha_,
    max_interp_area_m2_, temporal_alpha_height_, temporal_alpha_conf_, observation_timeout_);
  for (const auto & p : pts) {
    grid->accumulatePoint(p);
  }
  grid->finalizeFrame(this->now().seconds());
  return grid;
}

void ObstacleDetectorNode::classifyPoints(
  const std::vector<obstacle_detector::PointXYZ> & pts,
  const std::unordered_map<std::string, ColorInfo> & point_meta,
  obstacle_detector::GridHeightMap & grid,

  std::vector<obstacle_detector::PointXYZ> & ground_pts,
  std::vector<obstacle_detector::PointXYZ> & non_ground_pts,
  std::unordered_map<int, std::pair<double, Eigen::Vector3d>> & pca_results)
{
  for (const auto & p : pts) {
    int ix = static_cast<int>(std::floor((p.x - roi_x_min_) / grid_cell_size_));
    int iy = static_cast<int>(std::floor((p.y - roi_y_min_) / grid_cell_size_));
    obstacle_detector::GridCell cell;
    double slope_deg = 0.0;
    if (slope_method_ == "pca") {
      int idx = ix * grid.cols() + iy;
      auto itp = pca_results.find(idx);
      if (itp != pca_results.end()) {
        slope_deg = itp->second.first;
      } else {
        grid.getCellSlopeDeg(ix, iy, slope_deg);
      }
    } else {
      grid.getCellSlopeDeg(ix, iy, slope_deg);
    }
    if (grid.getCell(ix, iy, cell) && cell.has_observation) {
      auto itmeta = point_meta.find(make_key(p.x, p.y, p.z));
      if (itmeta != point_meta.end() && itmeta->second.has_intensity) {
        double in = itmeta->second.intensity;
        cell.confidence = std::min(1.0, cell.confidence * (1.0 - 0.2) + in * 0.2);
      }
      if (itmeta != point_meta.end() && itmeta->second.has_rgb && use_rgb_) {
        double r, g, b, h, s, v;
        unpackFloatRGB(itmeta->second.rgb, r, g, b);
        rgbToHsv(r, g, b, h, s, v);
        auto vh_param = this->get_parameter_or<std::vector<int64_t>>(
          "vegetation_h_range", std::vector<int64_t>{35, 85});
        std::vector<int> vh;
        vh.reserve(vh_param.size());
        for (auto vv : vh_param) vh.push_back(static_cast<int>(vv));
        if (vh.size() < 2) vh = {35, 85};
        double vs_min = this->get_parameter_or<double>("vegetation_v_min", 0.2);
        double vs_smin = this->get_parameter_or<double>("vegetation_s_min", 0.3);
        if (h >= vh[0] && h <= vh[1] && s >= vs_smin && v >= vs_min) {
          cell.confidence *= 0.5;
        }
      }
      double dz = std::abs(p.z - cell.height_median);
      double score =
        ground_separator_->computeGroundScore(slope_deg, cell.height_variance, cell.confidence);
      double alpha = ground_separator_->getEmaAlpha();
      cell.ground_ema = alpha * score + (1.0 - alpha) * cell.ground_ema;
      double high_th = ground_separator_->getHighThreshold();
      double low_th = ground_separator_->getLowThreshold();
      bool is_ground_by_score =
        (cell.ground_ema >= high_th) || (cell.ground_ema > low_th && cell.is_ground);
      if (dz <= ground_max_distance_ && is_ground_by_score) {
        ground_pts.push_back(p);
        continue;
      }
    }
    non_ground_pts.push_back(p);
  }
}

void ObstacleDetectorNode::publishConfidenceCloud(
  const obstacle_detector::GridHeightMap & grid,
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf)
{
  if (!(pub_confidence_cloud_ && pub_confidence_cloud_->get_subscription_count() > 0)) return;
  std::vector<obstacle_detector::PointXYZ> cells_pts;
  std::vector<float> cells_conf;
  for (int ix = 0; ix < grid.rows(); ++ix) {
    for (int iy = 0; iy < grid.cols(); ++iy) {
      obstacle_detector::GridCell c;
      if (!grid.getCell(ix, iy, c) || !c.has_observation) continue;
      double cx = roi_x_min_ + (ix + 0.5) * grid_cell_size_;
      double cy = roi_y_min_ + (iy + 0.5) * grid_cell_size_;
      double cz = c.height_median;
      cells_pts.push_back(obstacle_detector::PointXYZ{
        static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz)});
      cells_conf.push_back(static_cast<float>(c.confidence));
    }
  }
  if (!cells_pts.empty()) {
    auto out_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out_cloud->header = cloud_in_tf->header;
    out_cloud->header.frame_id = target_frame_;
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
    pub_confidence_cloud_->publish(*out_cloud);
  }
}

void ObstacleDetectorNode::publishObstacleCloudAndScan(
  const std::vector<obstacle_detector::PointXYZ> & obstacle_pts,
  const std::unordered_map<std::string, ColorInfo> & point_meta,
  const sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in_tf)
{
  if (!obstacle_pts.empty()) {
    auto out_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    out_cloud->header = cloud_in_tf->header;
    out_cloud->header.frame_id = target_frame_;
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
    bool include_rgb = use_rgb_ && ([&cloud_in_tf]() {
                         for (const auto & ff : cloud_in_tf->fields) {
                           if (ff.name == "rgb" || ff.name == "rgba") return true;
                         }
                         return false;
                       }());
    bool include_intensity = use_intensity_ && ([&cloud_in_tf]() {
                               for (const auto & ff : cloud_in_tf->fields) {
                                 if (ff.name == "intensity") return true;
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
      orgb = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*out_cloud, "rgb");
    if (include_intensity)
      oint = std::make_unique<sensor_msgs::PointCloud2Iterator<float>>(*out_cloud, "intensity");
    for (size_t i = 0; i < obstacle_pts.size(); ++i, ++ox, ++oy, ++oz) {
      const auto & p = obstacle_pts[i];
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
    pub_obstacle_cloud_->publish(*out_cloud);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
