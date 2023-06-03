#include "horiokart_drivers/motor_driver_node.hpp"

using horiokart_drivers::MotorDriverNode;

MotorDriverNode::MotorDriverNode(
    rclcpp::NodeOptions options)
    : Node("motor_driver_node", options)
{
    init_ros_params();
    prepare_ros_communications();

    motor_driver_ = std::make_shared<horiokart_drivers::MotorDriver>(device_name_);
}

void MotorDriverNode::init_ros_params()
{
    // Declare parameters
    this->declare_parameter<std::string>("motor_driver.device_name");
    this->declare_parameter<std::string>("motor_driver.twist_frame_id");
    this->declare_parameter<std::string>("motor_driver.base_frame_id");
    this->declare_parameter<double>("motor_driver.wheel_pitch");
    this->declare_parameter<double>("motor_driver.max_speed");

    // Get parameters
    this->get_parameter_or<std::string>(
        "motor_driver.device_name",
        device_name_,
        std::string("/dev/ttyUSB0"));

    this->get_parameter_or<std::string>(
        "motor_driver.twist_frame_id",
        twist_frame_id_,
        std::string("cmd_vel"));

    this->get_parameter_or<std::string>(
        "motor_driver.base_frame_id",
        base_frame_id_,
        std::string("base_footprint"));

    this->get_parameter_or<double>(
        "motor_driver.wheel_pitch",
        wheel_pitch_,
        0.358);

    this->get_parameter_or<double>(
        "motor_driver.max_speed",
        max_speed_,
        0.4);
}

void MotorDriverNode::prepare_ros_communications()
{
    // Create subscriber
    this->twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_frame_id_,
        rclcpp::QoS(1),
        std::bind(&MotorDriverNode::twist_sub_cb, this, std::placeholders::_1));
}

horiokart_drivers::SpeedParameter MotorDriverNode::create_speed_parameter(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    SpeedParameter req;

    // Convert twist to speed
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    if (linear_x > max_speed_)
    {
        linear_x = max_speed_;
    }
    else if (linear_x < -max_speed_)
    {
        linear_x = -max_speed_;
    }

    double left_speed = linear_x - angular_z * wheel_pitch_ / 2.0;
    double right_speed = linear_x + angular_z * wheel_pitch_ / 2.0;

    // Set speed
    req.left_wheel_speed = static_cast<int16_t>(left_speed * 1000.0);   // [mm/s]
    req.right_wheel_speed = static_cast<int16_t>(right_speed * 1000.0); // [mm/s]

    return req;
}

void MotorDriverNode::twist_sub_cb(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    SpeedParameter req = create_speed_parameter(msg);

    // TODO: print log about req 
    MotorDriverResponse res = motor_driver_->send_speed_command(req);
    // TODO: print log about res

    if (res.error != SerialError::NO_ERROR)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to send speed command to motor driver: %s",
                     SerialErrorStrings[static_cast<int>(res.error)].c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    // options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MotorDriverNode>(options);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}