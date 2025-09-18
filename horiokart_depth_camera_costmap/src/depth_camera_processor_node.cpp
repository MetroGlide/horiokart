#include "horiokart_depth_camera_costmap/core_processor.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <mutex>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std::chrono_literals;
using namespace horiokart::depth_camera_costmap;

class DepthCameraProcessorNode : public rclcpp::Node
{
public:
    DepthCameraProcessorNode()
        : Node("depth_camera_processor_node")
    {
        this->declare_parameter<std::string>("input_topic", "/camera/depth/color/points");
        this->declare_parameter<std::string>("output_topic", "/depth_costmap/occupancy_grid");
        this->declare_parameter<std::string>("marker_topic", "/depth_costmap/clusters");
        this->declare_parameter<std::string>("target_frame", "base_link");

        // Core parameters (exposed as ROS params)
        this->declare_parameter<double>("grid_resolution_m", 0.05);
        this->declare_parameter<double>("voxel_leaf_size_m", 0.02);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        this->declare_parameter<int>("normal_k", 10);
        this->declare_parameter<double>("max_slope_angle_deg", 30.0);
        this->declare_parameter<double>("normal_angle_threshold_deg", 10.0);
        this->declare_parameter<double>("max_step_height_m", 0.10);
        this->declare_parameter<double>("z_variance_threshold", 0.02);
        this->declare_parameter<int>("cost_traversable", 0);
        this->declare_parameter<int>("cost_semi_traversable", 50);
        this->declare_parameter<int>("cost_obstacle", 150);
        this->declare_parameter<int>("cost_lethal", 254); // prefer 254 (255 reserved for unknown)

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Read core params into CoreParams struct
        CoreParams core_defaults; // to get type defaults
        CoreParams core_params;
        core_params.grid_resolution_m = this->get_parameter("grid_resolution_m").as_double();
        core_params.voxel_leaf_size_m = this->get_parameter("voxel_leaf_size_m").as_double();
        core_params.sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        core_params.sor_stddev_mul_thresh = this->get_parameter("sor_stddev_mul_thresh").as_double();
        core_params.normal_k = this->get_parameter("normal_k").as_int();
        core_params.max_slope_angle_deg = this->get_parameter("max_slope_angle_deg").as_double();
        core_params.normal_angle_threshold_deg = this->get_parameter("normal_angle_threshold_deg").as_double();
        core_params.max_step_height_m = this->get_parameter("max_step_height_m").as_double();
        core_params.z_variance_threshold = this->get_parameter("z_variance_threshold").as_double();
        core_params.cost_traversable = static_cast<uint8_t>(this->get_parameter("cost_traversable").as_int());
        core_params.cost_semi_traversable = static_cast<uint8_t>(this->get_parameter("cost_semi_traversable").as_int());
        core_params.cost_obstacle = static_cast<uint8_t>(this->get_parameter("cost_obstacle").as_int());
        core_params.cost_lethal = static_cast<uint8_t>(this->get_parameter("cost_lethal").as_int());

        // store core_params for use in callback
        core_params_ = core_params;

        // register parameter update callback so CoreParams can be changed at runtime
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DepthCameraProcessorNode::onParametersUpdated, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&DepthCameraProcessorNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DepthCameraProcessorNode started, subscribing to %s", input_topic_.c_str());
    }

private:
    // Parameter update with validation: build a temp CoreParams, validate, then apply atomically
    rcl_interfaces::msg::SetParametersResult onParametersUpdated(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        res.reason = "";

        CoreParams temp;
        {
            std::lock_guard<std::mutex> lock(core_params_mutex_);
            temp = core_params_;
        }

        std::string err;
        for (const auto &p : params)
        {
            const auto &name = p.get_name();
            try
            {
                if (name == "grid_resolution_m")
                {
                    double v = p.as_double();
                    if (!(v > 0.0))
                        err += "grid_resolution_m must be > 0; ";
                    temp.grid_resolution_m = v;
                }
                else if (name == "voxel_leaf_size_m")
                {
                    double v = p.as_double();
                    if (!(v > 0.0))
                        err += "voxel_leaf_size_m must be > 0; ";
                    temp.voxel_leaf_size_m = v;
                }
                else if (name == "sor_mean_k")
                {
                    int v = p.as_int();
                    if (v < 0)
                        err += "sor_mean_k must be >= 0; ";
                    temp.sor_mean_k = v;
                }
                else if (name == "sor_stddev_mul_thresh")
                {
                    double v = p.as_double();
                    if (v < 0.0)
                        err += "sor_stddev_mul_thresh must be >= 0; ";
                    temp.sor_stddev_mul_thresh = v;
                }
                else if (name == "normal_k")
                {
                    int v = p.as_int();
                    if (v <= 0)
                        err += "normal_k must be > 0; ";
                    temp.normal_k = v;
                }
                else if (name == "max_slope_angle_deg")
                {
                    double v = p.as_double();
                    if (!(v >= 0.0 && v <= 90.0))
                        err += "max_slope_angle_deg must be in [0,90]; ";
                    temp.max_slope_angle_deg = v;
                }
                else if (name == "normal_angle_threshold_deg")
                {
                    double v = p.as_double();
                    if (v < 0.0)
                        err += "normal_angle_threshold_deg must be >= 0; ";
                    temp.normal_angle_threshold_deg = v;
                }
                else if (name == "max_step_height_m")
                {
                    double v = p.as_double();
                    if (v < 0.0)
                        err += "max_step_height_m must be >= 0; ";
                    temp.max_step_height_m = v;
                }
                else if (name == "z_variance_threshold")
                {
                    double v = p.as_double();
                    if (v < 0.0)
                        err += "z_variance_threshold must be >= 0; ";
                    temp.z_variance_threshold = v;
                }
                else if (name == "cost_traversable")
                {
                    int v = p.as_int();
                    if (v < 0 || v > 254)
                        err += "cost_traversable must be in [0,254]; ";
                    temp.cost_traversable = static_cast<uint8_t>(v);
                }
                else if (name == "cost_semi_traversable")
                {
                    int v = p.as_int();
                    if (v < 0 || v > 254)
                        err += "cost_semi_traversable must be in [0,254]; ";
                    temp.cost_semi_traversable = static_cast<uint8_t>(v);
                }
                else if (name == "cost_obstacle")
                {
                    int v = p.as_int();
                    if (v < 0 || v > 254)
                        err += "cost_obstacle must be in [0,254]; ";
                    temp.cost_obstacle = static_cast<uint8_t>(v);
                }
                else if (name == "cost_lethal")
                {
                    int v = p.as_int();
                    // 255 is reserved for 'unknown' sentinel in core. Reject if user tries to set 255.
                    if (v < 0 || v > 254)
                        err += "cost_lethal must be in [0,254] (255 reserved); ";
                    temp.cost_lethal = static_cast<uint8_t>(v);
                }
                else
                {
                    // ignore unrelated parameters
                }
            }
            catch (const std::exception &e)
            {
                err += std::string("Invalid type for parameter: ") + name + "; ";
            }
        }

        if (!err.empty())
        {
            res.successful = false;
            res.reason = err;
            RCLCPP_WARN(this->get_logger(), "Parameter update rejected: %s", res.reason.c_str());
            return res;
        }

        // Additional cross-parameter sanity checks
        if (temp.voxel_leaf_size_m > temp.grid_resolution_m)
        {
            res.successful = false;
            res.reason = "voxel_leaf_size_m should be <= grid_resolution_m";
            RCLCPP_WARN(this->get_logger(), "%s", res.reason.c_str());
            return res;
        }

        // All validations passed; apply atomically
        {
            std::lock_guard<std::mutex> lock(core_params_mutex_);
            core_params_ = temp;
        }

        RCLCPP_INFO(this->get_logger(), "CoreParams updated via parameter callback");
        return res;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // TF lookup and transform to target_frame_
        geometry_msgs::msg::TransformStamped tfst;
        try
        {
            tfst = tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, tf2::TimePointZero);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", e.what());
            return;
        }

        // convert to PCL and transform
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);
        Eigen::Affine3d T = tf2::transformToEigen(tfst.transform);
        pcl::transformPointCloud(pcl_cloud, pcl_cloud, T);

        // convert to core Point3D vector
        std::vector<Point3D> pts;
        pts.reserve(pcl_cloud.size());
        for (const auto &p : pcl_cloud)
        {
            Point3D cp;
            cp.x = p.x;
            cp.y = p.y;
            cp.z = p.z;
            cp.r = p.r;
            cp.g = p.g;
            cp.b = p.b;
            pts.push_back(cp);
        }

        // use stored core_params_ (make a local copy under lock to avoid holding mutex during processing)
        CoreParams local_params;
        {
            std::lock_guard<std::mutex> lock(core_params_mutex_);
            local_params = core_params_;
        }

        GridCostMap grid = processPointCloud(pts, local_params);
        auto arr = convertToOccupancyArray(grid, OccupancyOptions());

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = this->now();
        og.header.frame_id = grid.frame_id.empty() ? target_frame_ : grid.frame_id;
        og.info.resolution = grid.resolution_m;
        og.info.width = grid.width;
        og.info.height = grid.height;
        og.info.origin.position.x = grid.origin.x;
        og.info.origin.position.y = grid.origin.y;
        og.info.origin.position.z = grid.origin.z;
        og.data.resize(arr.size());
        for (size_t i = 0; i < arr.size(); ++i)
        {
            uint8_t v = arr[i];
            if (v == 255)
                og.data[i] = -1;
            else
                og.data[i] = static_cast<int8_t>(std::round((v / 254.0) * 100.0));
        }
        pub_->publish(og);

        // clusters -> MarkerArray
        ClusterParams cparams; // defaults
        auto clusters = clusterCostMap(grid, cparams);
        visualization_msgs::msg::MarkerArray ma;
        int id = 0;
        for (const auto &c : clusters)
        {
            visualization_msgs::msg::Marker m;
            m.header = og.header;
            m.ns = "depth_clusters";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = grid.origin.x + (c.centroid.x() + 0.5f) * grid.resolution_m;
            m.pose.position.y = grid.origin.y + (c.centroid.y() + 0.5f) * grid.resolution_m;
            m.pose.position.z = 0.1;
            m.scale.x = m.scale.y = m.scale.z = 0.1;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 0.0;
            m.color.a = 1.0;
            ma.markers.push_back(m);
        }
        marker_pub_->publish(ma);
    }

    std::string input_topic_, output_topic_, marker_topic_, target_frame_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    CoreParams core_params_;
    std::mutex core_params_mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthCameraProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
