#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class OdomCovarianceOverrideNode(Node):
    def __init__(self):
        super().__init__('odom_covariance_override_node')
        self.get_logger().info('odom_covariance_override_node started')
        self.init_parameters()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            1)

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom/covariance',
            1)

    def init_parameters(self):
        # self._odom_pose_covariance = [0.0] * 36
        # self._odom_pose_covariance[0] = 0.1
        # self._odom_pose_covariance[7] = 0.1
        # self._odom_pose_covariance[14] = 0.1
        # self._odom_pose_covariance[21] = 0.1
        # self._odom_pose_covariance[28] = 0.1
        # self._odom_pose_covariance[35] = 0.1

        # self._odom_twist_covariance = [0.0] * 36
        # self._odom_twist_covariance[0] = 0.1
        # self._odom_twist_covariance[7] = 0.1
        # self._odom_twist_covariance[14] = 0.1
        # self._odom_twist_covariance[21] = 0.1
        # self._odom_twist_covariance[28] = 0.1
        # self._odom_twist_covariance[35] = 0.1

        self._odom_pose_covariance = [
            # 6.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            # 0.0, 6.5, 0.0, 0.0, 0.0, 0.0,
            10.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.4,
        ]
        self._odom_twist_covariance = [
            0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.78,
        ]

    def odom_callback(self, msg):
        msg.pose.covariance = self._odom_pose_covariance
        msg.twist.covariance = self._odom_twist_covariance

        self.odom_pub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = OdomCovarianceOverrideNode()
    rclpy.spin(node)
    rclpy.shutdown()
