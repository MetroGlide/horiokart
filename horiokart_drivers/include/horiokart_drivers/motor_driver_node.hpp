// ROS2 node of publish wheel odometry
#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
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
#include "horiokart_drivers/devices/motor_driver.hpp"

namespace horiokart_drivers
{
    class MotorDriverNode : public rclcpp::Node
    {
    public:
        explicit MotorDriverNode(
            rclcpp::NodeOptions options);
        virtual ~MotorDriverNode(){};

    private:
        void init_ros_params();
        void prepare_ros_communications();

        void twist_sub_cb(
            const geometry_msgs::msg::Twist::SharedPtr msg);

        void emergency_stop_sub_cb(
            const std_msgs::msg::Bool::SharedPtr msg);

        std::shared_ptr<horiokart_drivers::MotorDriver> motor_driver_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;

        std::string device_name_;

        std::string twist_frame_id_;
        std::string base_frame_id_;

        double wheel_pitch_; // [m]
        double max_speed_;   // [m/s]

        bool emergency_stop_;

        SpeedParameter create_speed_parameter(
            const geometry_msgs::msg::Twist::SharedPtr msg);
    };

}
