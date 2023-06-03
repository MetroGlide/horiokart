#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped


class OdometryTFBroadcasterNode(Node):

    def __init__(self):
        super().__init__('odometry_tf_broadcaster_node')
        self.get_logger().info('odometry_tf_broadcaster_node started')

        self.odom_frame_id = self.declare_parameter(
            'odom_frame_id',
            'odom').value
        self.child_frame_id = self.declare_parameter(
            'child_frame_id',
            'base_footprint').value

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            1)
        self.odom_sub  # prevent unused variable warning

        self.br = TransformBroadcaster(self)

    def odom_callback(self, msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    odometry_tf_broadcaster_node = OdometryTFBroadcasterNode()

    rclpy.spin(odometry_tf_broadcaster_node)

    odometry_tf_broadcaster_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
