// ROS2 node of publish wheel odometry
#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

// C++
#include <memory>
#include <string>
#include <cmath>

// horiokart
#include "horiokart_drivers/sensors/wheel_odometry.hpp"

namespace horiokart_drivers
{
    class WheelOdometryNode : public rclcpp::Node
    {
    public:
        explicit WheelOdometryNode(
            rclcpp::NodeOptions options);
        virtual ~WheelOdometryNode(){};

    private:
        void init_ros_params();

        void prepare_ros_communications();
        void publish();

        void update_odometry();
        void publish_odometry();
        void publish_tf();

        bool zero_reset();

        void zero_reset_srv_callback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        void error_recovery();

        std::shared_ptr<horiokart_drivers::WheelOdometry> wheel_odometry_;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_reset_srv_;

        rclcpp::TimerBase::SharedPtr publish_timer_;

        std::string frame_id_of_odometry_;
        std::string child_frame_id_of_odometry_;

        std::string device_name_;

        int publish_rate_;
        bool publish_tf_;
        bool always_publish_;

        bool inv_x_, inv_y_, inv_th_;

        OdometryData last_valid_data_, current_data_;

        int error_count_ = 0;
        int error_recovery_count_; 
    };

}