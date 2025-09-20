#include "horiokart_depth_camera_costmap/core_processor.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
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
        this->declare_parameter<std::string>("target_frame", "");

        // Core parameters (exposed as ROS params)
        this->declare_parameter<double>("grid_resolution", 0.05);
        this->declare_parameter<double>("voxel_size", 0.02);
        this->declare_parameter<int>("sor_mean_k", 50);
        this->declare_parameter<double>("sor_stddev_mul_thresh", 1.0);
        this->declare_parameter<int>("normal_k", 10);
        this->declare_parameter<double>("z_variance_threshold", 0.01);
        this->declare_parameter<int>("cost_lethal", 254);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        marker_topic_ = this->get_parameter("marker_topic").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        // Read core params into CoreParams struct
        CoreParams core_params;
        core_params.grid_resolution = static_cast<float>(this->get_parameter("grid_resolution").as_double());
        core_params.voxel_size = static_cast<float>(this->get_parameter("voxel_size").as_double());
        core_params.sor_mean_k = this->get_parameter("sor_mean_k").as_int();
        core_params.sor_stddev_mul_thresh = static_cast<float>(this->get_parameter("sor_stddev_mul_thresh").as_double());
        core_params.normal_k = this->get_parameter("normal_k").as_int();
        core_params.z_variance_threshold = static_cast<float>(this->get_parameter("z_variance_threshold").as_double());
        core_params.cost_lethal = static_cast<std::uint8_t>(this->get_parameter("cost_lethal").as_int());

        core_params_ = core_params;

        // register parameter update callback so CoreParams can be changed at runtime
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DepthCameraProcessorNode::onParametersUpdated, this, std::placeholders::_1));

        pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10,
            std::bind(&DepthCameraProcessorNode::pointCloudCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "DepthCameraProcessorNode started, subscribing to %s", input_topic_.c_str());
    }

private:
    // Parameter update with validation
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
                if (name == "grid_resolution")
                {
                    double v = p.as_double();
                    if (!(v > 0.0))
                        err += "grid_resolution must be > 0; ";
                    temp.grid_resolution = static_cast<float>(v);
                }
                else if (name == "voxel_size")
                {
                    double v = p.as_double();
                    if (!(v > 0.0))
                        err += "voxel_size must be > 0; ";
                    temp.voxel_size = static_cast<float>(v);
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
                    temp.sor_stddev_mul_thresh = static_cast<float>(v);
                }
                else if (name == "normal_k")
                {
                    int v = p.as_int();
                    if (v <= 0)
                        err += "normal_k must be > 0; ";
                    temp.normal_k = v;
                }
                else if (name == "z_variance_threshold")
                {
                    double v = p.as_double();
                    if (v < 0.0)
                        err += "z_variance_threshold must be >= 0; ";
                    temp.z_variance_threshold = static_cast<float>(v);
                }
                else if (name == "cost_lethal")
                {
                    int v = p.as_int();
                    if (v < 0 || v > 254)
                        err += "cost_lethal must be in [0,254] (255 reserved); ";
                    temp.cost_lethal = static_cast<std::uint8_t>(v);
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

        if (temp.voxel_size > temp.grid_resolution)
        {
            res.successful = false;
            res.reason = "voxel_size should be <= grid_resolution";
            RCLCPP_WARN(this->get_logger(), "%s", res.reason.c_str());
            return res;
        }

        {
            std::lock_guard<std::mutex> lock(core_params_mutex_);
            core_params_ = temp;
        }

        RCLCPP_INFO(this->get_logger(), "CoreParams updated via parameter callback");
        return res;
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // convert to Point3D vector (no color)
        std::vector<Point3D> pts;
        pts.reserve(msg->width * msg->height);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (size_t i = 0; i < static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height); ++i, ++it_x, ++it_y, ++it_z)
        {
            float x = *it_x;
            float y = *it_y;
            float z = *it_z;
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            pts.push_back(Point3D{x, y, z});
        }

        CoreParams local_params;
        {
            std::lock_guard<std::mutex> lock(core_params_mutex_);
            local_params = core_params_;
        }

        GridCostMap grid = processPointCloud(pts, local_params);
        auto arr = convertToOccupancyArray(grid);

        nav_msgs::msg::OccupancyGrid og;
        og.header.stamp = this->now();
        // prefer explicit target_frame if set, else use input frame
        og.header.frame_id = (!target_frame_.empty()) ? target_frame_ : msg->header.frame_id;
        og.info.resolution = grid.resolution;
        og.info.width = static_cast<unsigned int>(grid.width);
        og.info.height = static_cast<unsigned int>(grid.height);
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
                og.data[i] = static_cast<int8_t>(std::round((v / 254.0f) * 100.0f));
        }
        pub_->publish(og);

        // publish clusters as simple markers
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
            m.pose.position.x = c.centroid.x;
            m.pose.position.y = c.centroid.y;
            m.pose.position.z = c.centroid.z;
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
