#include "horiokart_drivers/wheel_odometry_node.hpp"

using horiokart_drivers::WheelOdometryNode;

WheelOdometryNode::WheelOdometryNode(
    rclcpp::NodeOptions options) : Node("wheel_odometry_node", options)
{
    RCLCPP_INFO(this->get_logger(), "wheel_odometry_node has been created.");

    init_ros_params();

    // Create wheel odometry object
    wheel_odometry_ = std::make_shared<horiokart_drivers::WheelOdometry>(device_name_);
    zero_reset();

    prepare_ros_communications();
}

void WheelOdometryNode::init_ros_params()
{
    // Declare parameters
    this->declare_parameter<std::string>("odometry.frame_id");
    this->declare_parameter<std::string>("odometry.child_frame_id");
    this->declare_parameter<bool>("odometry.publish_tf");
    this->declare_parameter<int>("odometry.publish_rate");

    this->declare_parameter<std::string>("odometry.device_name");

    this->declare_parameter<bool>("odometry.always_publish");

    this->declare_parameter<bool>("odometry.inv_x");
    this->declare_parameter<bool>("odometry.inv_y");
    this->declare_parameter<bool>("odometry.inv_th");

    this->declare_parameter<double>("odometry.covariance_x");
    this->declare_parameter<double>("odometry.covariance_y");
    this->declare_parameter<double>("odometry.covariance_yaw");
    this->declare_parameter<double>("odometry.covariance_vx");
    this->declare_parameter<double>("odometry.covariance_vyaw");

    this->declare_parameter<int>("odometry.error_recovery_count");

    // Get parameters
    this->get_parameter_or<std::string>(
        "odometry.frame_id",
        frame_id_of_odometry_,
        std::string("odom"));

    this->get_parameter_or<std::string>(
        "odometry.child_frame_id",
        child_frame_id_of_odometry_,
        std::string("base_footprint"));

    this->get_parameter_or<bool>(
        "odometry.publish_tf",
        publish_tf_,
        true);

    this->get_parameter_or<int>(
        "odometry.publish_rate",
        publish_rate_,
        20);

    this->get_parameter_or<std::string>(
        "odometry.device_name",
        device_name_,
        std::string("/dev/ttyUSB0"));

    this->get_parameter_or<bool>(
        "odometry.always_publish",
        always_publish_,
        false);

    this->get_parameter_or<bool>(
        "odometry.inv_x",
        inv_x_,
        false);

    this->get_parameter_or<bool>(
        "odometry.inv_y",
        inv_y_,
        false);

    this->get_parameter_or<bool>(
        "odometry.inv_th",
        inv_th_,
        false);

    this->get_parameter_or<int>(
        "odometry.error_recovery_count",
        error_recovery_count_,
        4);

    this->get_parameter_or<double>(
        "odometry.covariance_x",
        covariance_x_,
        10.0);

    this->get_parameter_or<double>(
        "odometry.covariance_y",
        covariance_y_,
        10.0);

    this->get_parameter_or<double>(
        "odometry.covariance_yaw",
        covariance_yaw_,
        0.4);

    this->get_parameter_or<double>(
        "odometry.covariance_vx",
        covariance_vx_,
        0.5);

    this->get_parameter_or<double>(
        "odometry.covariance_vyaw",
        covariance_vyaw_,
        0.78);
}

void WheelOdometryNode::prepare_ros_communications()
{
    // Create publisher
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(frame_id_of_odometry_, qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Create timer
    auto publish_callback = std::bind(&WheelOdometryNode::publish, this);
    auto period = std::chrono::milliseconds(1000 / publish_rate_);

    publish_timer_ = this->create_wall_timer(period, publish_callback);

    // Create service
}

void WheelOdometryNode::publish()
{
    update_odometry();

    publish_odometry();

    if (publish_tf_)
    {
        publish_tf();
    }
}

void WheelOdometryNode::update_odometry()
{
    current_data_ = wheel_odometry_->get_data();

    RCLCPP_DEBUG(this->get_logger(),
                 "RAW DATA: %s",
                 SerialCommunicator::format_hex(current_data_.raw).c_str());

    RCLCPP_DEBUG(this->get_logger(),
                 "x: %f, y: %f, th: %f, vx: %f, vy: %f, vth: %f",
                 current_data_.x,
                 current_data_.y,
                 current_data_.th,
                 current_data_.vx,
                 current_data_.vy,
                 current_data_.vth);

    if (current_data_.error == horiokart_drivers::SerialError::NO_ERROR)
    {
        if (inv_x_)
            current_data_.x *= -1;
        if (inv_y_)
            current_data_.y *= -1;
        if (inv_th_)
            current_data_.th *= -1;

        last_valid_data_ = current_data_;
        error_count_ = 0;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Serial error: %s",
                     horiokart_drivers::SerialErrorStrings[static_cast<int>(current_data_.error)].c_str());

        error_count_++;
        if (error_count_ >= error_recovery_count_)
        {
            error_recovery();
        }
    }
}

void WheelOdometryNode::publish_odometry()
{
    if (current_data_.error == horiokart_drivers::SerialError::NO_ERROR ||
        always_publish_)
    {

        auto odom_msg = nav_msgs::msg::Odometry();

        odom_msg.header.frame_id = frame_id_of_odometry_;
        odom_msg.header.stamp = this->now();

        odom_msg.child_frame_id = child_frame_id_of_odometry_;

        odom_msg.pose.pose.position.x = last_valid_data_.x;
        odom_msg.pose.pose.position.y = last_valid_data_.y;
        odom_msg.pose.pose.position.z = 0.0;

        // create quaternion from yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, last_valid_data_.th);

        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        double vx = sqrt(pow(last_valid_data_.vx, 2) + pow(last_valid_data_.vy, 2));
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = last_valid_data_.vth;

        odom_msg.pose.covariance[0] = covariance_x_;
        odom_msg.pose.covariance[7] = covariance_y_;
        odom_msg.pose.covariance[35] = covariance_yaw_;

        odom_msg.twist.covariance[0] = covariance_vx_;
        odom_msg.twist.covariance[35] = covariance_vyaw_;

        odom_pub_->publish(odom_msg);
    }
}

void WheelOdometryNode::publish_tf()
{
    if (current_data_.error == horiokart_drivers::SerialError::NO_ERROR ||
        always_publish_)
    {
        geometry_msgs::msg::TransformStamped odom_tf;

        odom_tf.header.stamp = this->now();
        odom_tf.header.frame_id = frame_id_of_odometry_;
        odom_tf.child_frame_id = child_frame_id_of_odometry_;

        odom_tf.transform.translation.x = last_valid_data_.x;
        odom_tf.transform.translation.y = last_valid_data_.y;
        odom_tf.transform.translation.z = 0.0;

        // create quaternion from yaw
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, last_valid_data_.th);

        odom_tf.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_tf);
    }
}

bool WheelOdometryNode::zero_reset()
{
    SerialError error = wheel_odometry_->send_zero_reset();
    if (error != SerialError::NO_ERROR)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Wheel odometry zero reset error: %s",
                     SerialErrorStrings[static_cast<int>(error)].c_str());
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Wheel odometry zero reset success!");
        return true;
    }
}

void WheelOdometryNode::zero_reset_srv_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (zero_reset())
    {
        response->success = true;
    }
    else
    {
        response->success = false;
    }
}

void WheelOdometryNode::error_recovery()
{
    RCLCPP_ERROR(this->get_logger(),
                 "Wheel odometry error recovery start");
    wheel_odometry_->reset_serial();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<WheelOdometryNode>(
        options);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}