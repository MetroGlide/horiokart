#include <memory>
#include <string>
#include <vector>

#include "obstacle_detection/obstacle_detector.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using std::placeholders::_1;

class ObstacleDetection3DNode : public rclcpp::Node
{
public:
  ObstacleDetection3DNode() : Node("obstacle_detection_3d_node")
  {
    // Declare parameters
    this->declare_parameter<double>("voxel_leaf_size", 0.05);
    this->declare_parameter<double>("cropbox_x_min", 0.0);
    this->declare_parameter<double>("cropbox_x_max", 3.0);
    this->declare_parameter<double>("cropbox_y_min", -0.75);
    this->declare_parameter<double>("cropbox_y_max", 0.75);
    this->declare_parameter<double>("cropbox_z_min", -0.1);
    this->declare_parameter<double>("cropbox_z_max", 1.0);
    this->declare_parameter<double>("grid_size", 0.05);
    this->declare_parameter<double>("delta_z_threshold", 0.15);
    this->declare_parameter<double>("ror_radius_search", 0.2);
    this->declare_parameter<int>("ror_min_neighbors", 3);
    this->declare_parameter<double>("cluster_tolerance", 0.3);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 1000);

    obstacle_detection::ObstacleDetectionParams p;
    p.voxel_leaf_size = this->get_parameter("voxel_leaf_size").as_double();
    p.cropbox_x_min = this->get_parameter("cropbox_x_min").as_double();
    p.cropbox_x_max = this->get_parameter("cropbox_x_max").as_double();
    p.cropbox_y_min = this->get_parameter("cropbox_y_min").as_double();
    p.cropbox_y_max = this->get_parameter("cropbox_y_max").as_double();
    p.cropbox_z_min = this->get_parameter("cropbox_z_min").as_double();
    p.cropbox_z_max = this->get_parameter("cropbox_z_max").as_double();
    p.grid_size = this->get_parameter("grid_size").as_double();
    p.delta_z_threshold = this->get_parameter("delta_z_threshold").as_double();
    p.ror_radius_search = this->get_parameter("ror_radius_search").as_double();
    p.ror_min_neighbors = this->get_parameter("ror_min_neighbors").as_int();
    p.cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
    p.min_cluster_size = this->get_parameter("min_cluster_size").as_int();
    p.max_cluster_size = this->get_parameter("max_cluster_size").as_int();

    detector_ = std::make_unique<obstacle_detection::ObstacleDetector>(p);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    
    pub_obstacle_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/points_obstacle", qos);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/cluster_markers", qos);
    
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", qos, std::bind(&ObstacleDetection3DNode::pointcloudCallback, this, _1));
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 1. TF transformation
    sensor_msgs::msg::PointCloud2 transformed_msg;
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "base_link", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*msg, transformed_msg, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform %s to base_link: %s",
                  msg->header.frame_id.c_str(), ex.what());
      return;
    }

    // 2. Convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
    pcl::fromROSMsg(transformed_msg, input_cloud);

    // 3. Process
    pcl::PointCloud<pcl::PointXYZRGB> obstacle_cloud;
    std::vector<pcl::PointIndices> clusters;
    
    if (!detector_->process(input_cloud, obstacle_cloud, clusters)) {
        return;
    }

    // 4. Publish PointCloud2
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(obstacle_cloud, out_msg);
    out_msg.header.frame_id = "base_link";
    out_msg.header.stamp = msg->header.stamp;
    pub_obstacle_->publish(out_msg);

    // 5. Publish Markers
    publishMarkers(obstacle_cloud, clusters, msg->header.stamp);
  }

  void publishMarkers(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, 
                      const std::vector<pcl::PointIndices>& clusters,
                      const rclcpp::Time& stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Delete all previous markers
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    int id = 0;
    for (const auto& cluster : clusters) {
      if (cluster.indices.empty()) continue;

      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = std::numeric_limits<float>::max();
      float max_y = std::numeric_limits<float>::lowest();
      float min_z = std::numeric_limits<float>::max();
      float max_z = std::numeric_limits<float>::lowest();

      for (const auto& idx : cluster.indices) {
        const auto& p = cloud.points[idx];
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
        min_z = std::min(min_z, p.z);
        max_z = std::max(max_z, p.z);
      }

      visualization_msgs::msg::Marker bbox;
      bbox.header.frame_id = "base_link";
      bbox.header.stamp = stamp;
      bbox.ns = "obstacle_clusters";
      bbox.id = id++;
      bbox.type = visualization_msgs::msg::Marker::LINE_LIST;
      bbox.action = visualization_msgs::msg::Marker::ADD;
      bbox.pose.orientation.w = 1.0;
      bbox.scale.x = 0.02; // Line width
      bbox.color.r = 1.0;
      bbox.color.g = 0.0;
      bbox.color.b = 0.0;
      bbox.color.a = 1.0;
      bbox.lifetime = rclcpp::Duration::from_seconds(0.5);

      // 8 corners of the bounding box
      geometry_msgs::msg::Point p1, p2, p3, p4, p5, p6, p7, p8;
      p1.x = min_x; p1.y = min_y; p1.z = min_z;
      p2.x = max_x; p2.y = min_y; p2.z = min_z;
      p3.x = max_x; p3.y = max_y; p3.z = min_z;
      p4.x = min_x; p4.y = max_y; p4.z = min_z;
      p5.x = min_x; p5.y = min_y; p5.z = max_z;
      p6.x = max_x; p6.y = min_y; p6.z = max_z;
      p7.x = max_x; p7.y = max_y; p7.z = max_z;
      p8.x = min_x; p8.y = max_y; p8.z = max_z;

      // Bottom rectangle
      bbox.points.push_back(p1); bbox.points.push_back(p2);
      bbox.points.push_back(p2); bbox.points.push_back(p3);
      bbox.points.push_back(p3); bbox.points.push_back(p4);
      bbox.points.push_back(p4); bbox.points.push_back(p1);

      // Top rectangle
      bbox.points.push_back(p5); bbox.points.push_back(p6);
      bbox.points.push_back(p6); bbox.points.push_back(p7);
      bbox.points.push_back(p7); bbox.points.push_back(p8);
      bbox.points.push_back(p8); bbox.points.push_back(p5);

      // Vertical lines
      bbox.points.push_back(p1); bbox.points.push_back(p5);
      bbox.points.push_back(p2); bbox.points.push_back(p6);
      bbox.points.push_back(p3); bbox.points.push_back(p7);
      bbox.points.push_back(p4); bbox.points.push_back(p8);

      marker_array.markers.push_back(bbox);
      
      // Text label
      visualization_msgs::msg::Marker text;
      text.header.frame_id = "base_link";
      text.header.stamp = stamp;
      text.ns = "obstacle_clusters_text";
      int cluster_num = id / 2 + 1;  // bbox と text で 2ずつ増加するためクラスタ番号は id/2+1
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = (min_x + max_x) / 2.0;
      text.pose.position.y = (min_y + max_y) / 2.0;
      text.pose.position.z = max_z + 0.1;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.1; // Text height
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      text.text = "Cluster " + std::to_string(cluster_num) + " (" + std::to_string(cluster.indices.size()) + " pts)";
      text.lifetime = rclcpp::Duration::from_seconds(0.5);
      
      marker_array.markers.push_back(text);
    }
    
    pub_markers_->publish(marker_array);
  }

  std::unique_ptr<obstacle_detection::ObstacleDetector> detector_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetection3DNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
