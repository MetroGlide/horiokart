#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

import dataclasses


@dataclasses.dataclass
class BaseOdometry:
    x: float
    y: float
    z: float
    q: list


class OdomOffsetRepublisherNode(Node):

    def __init__(self):
        super().__init__('odom_offset_republisher_node')
        self.get_logger().info('odom_offset_republisher_node started')
        self.init_parameters()

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_raw',
            self.odom_callback,
            1)
        self.odom_sub  # prevent unused variable warning

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            1)

    def init_parameters(self):
        self.base_odom = None 

    def odom_callback(self, msg):
        if self.base_odom is None:
            self.base_odom = BaseOdometry(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                z=msg.pose.pose.position.z,
                q=[msg.pose.pose.orientation.x,
                   msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z,
                   -msg.pose.pose.orientation.w]
            )

        msg.pose.pose.position.x -= self.base_odom.x
        msg.pose.pose.position.y -= self.base_odom.y
        msg.pose.pose.position.z -= self.base_odom.z

        current_q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        q = quaternion_multiply(current_q, self.base_odom.q)
        # msg.pose.pose.orientation.x = q[0]
        # msg.pose.pose.orientation.y = q[1]
        # msg.pose.pose.orientation.z = q[2]
        # msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    odom_offset_republisher_node = OdomOffsetRepublisherNode()

    rclpy.spin(odom_offset_republisher_node)

    odom_offset_republisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
