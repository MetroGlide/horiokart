#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

import tf2_ros
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from gps_transform import GpsTransform
from gps_transform import Pose as GpsPose


class GpsTransformNode(Node):
    def __init__(self):
        super().__init__('gps_transform_node')
        self.get_logger().info('gps_transform_node started')
        self.init_parameters()

        self.gps_sub = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            1)
        self.gps_sub

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom/gps',
            1)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = ''

        self.current_odom_msg = None
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            1)

        self.set_resistration_mode_srv = self.create_service(
            Trigger,
            'set_resistration_mode',
            self.set_resistration_mode_callback)

    def init_parameters(self):
        self.resistration = False
        self._gps_transform = GpsTransform(offset_yaw=2.75)

        # prepare tf2 listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def gps_callback(self, msg):
        map_position = self._gps_transform.transform_to_map(
            msg.latitude, msg.longitude)

        if self.resistration:
            # listen tf from map to base_link
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().info('tf not found')
                return

            self.get_logger().info("-----")
            self.get_logger().info(
                f"tf: {transform.transform.translation.x}, {transform.transform.translation.y}, {euler_from_quaternion([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])[2]}")
            self.get_logger().info(
                f"map_position: {map_position.x}, {map_position.y}, {map_position.yaw}")
            self.get_logger().info(
                f"before offset yaw: {self._gps_transform.origin_utm.yaw}")

            self._gps_transform.feedback_pose(
                GpsPose(
                    x=transform.transform.translation.x,
                    y=transform.transform.translation.y,
                    yaw=euler_from_quaternion([
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    ])[2]
                )
            )
            self.get_logger().info('feedback pose to gps')
            self.get_logger().info(
                f"after offset yaw: {self._gps_transform.origin_utm.yaw}")

        self.odom_msg.header.stamp = msg.header.stamp
        self.odom_msg.pose.pose.position.x = map_position.x
        self.odom_msg.pose.pose.position.y = map_position.y
        self.odom_msg.pose.pose.position.z = 0.0
        # set orientation from yaw
        q = quaternion_from_euler(0.0, 0.0, map_position.yaw)
        self.odom_msg.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])

        # set covariance
        self.odom_msg.pose.covariance[0] = msg.position_covariance[0]
        self.odom_msg.pose.covariance[7] = msg.position_covariance[4]
        self.odom_msg.pose.covariance[14] = 0.0
        self.odom_msg.pose.covariance[21] = 0.0
        self.odom_msg.pose.covariance[28] = 0.0
        self.odom_msg.pose.covariance[35] = 0.0

        self.odom_pub.publish(self.odom_msg)

    def odom_callback(self, msg):
        self.current_odom_msg = msg

    def set_resistration_mode_callback(self, request, response):
        self.resistration = request.data
        response.success = True
        response.message = f'set resistration mode to {self.resistration}'
        return response


def main(args=None):
    rclpy.init(args=args)

    node = GpsTransformNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
