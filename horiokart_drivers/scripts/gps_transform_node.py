#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

import tf2_ros
import tf2_py
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from gps_transform import GpsTransform
from gps_transform import Pose as GpsPose


class GpsTransformNode(Node):
    def __init__(self):
        super().__init__('gps_transform_node')
        self.get_logger().info('gps_transform_node started')

        self.init_parameters()
        self.prepare_tf()
        self.init_ros_communication()

    def init_parameters(self):
        self.resistration = self.declare_parameter(
            'resistration',
            True).value
        self.map_frame_id = self.declare_parameter(
            'map_frame_id',
            'map').value
        self.gps_frame_id = self.declare_parameter(
            'gps_frame_id',
            'gps_link').value
        self.robot_base_frame_id = self.declare_parameter(
            'robot_base_frame_id',
            'base_link').value
        self.utm_param_yaml_path = self.declare_parameter(
            'utm_param_yaml_path',
            '/root/ros2_data/utm_to_map_refpoint.yaml').value

        self._gps_transform = GpsTransform(offset_yaw=2.75)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.map_frame_id
        self.odom_msg.child_frame_id = self.gps_frame_id

    def init_ros_communication(self):
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
        self.odom_pub

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

        self.save_utm_refpoints_srv = self.create_service(
            Trigger,
            'save_utm_refpoints',
            self.save_utm_refpoint_callback)

    def prepare_tf(self):
        # prepare tf2 listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        try:
            self._base_link_to_gps_transform = self._tf_buffer.lookup_transform(
                self.robot_base_frame_id,  # target frame
                self.gps_frame_id,  # source frame
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().info('tf not found')
            return

        self._gps_to_base_link_transform = tf2_py.inverse_transform(
            self._base_link_to_gps_transform)

    def gps_callback(self, msg):
        map_position = self._gps_transform.transform_to_map(
            msg.latitude, msg.longitude)

        if self.resistration:
            # listen tf from map to base_link
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame_id,  # target frame
                    self.gps_frame_id,  # source frame
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

            map_frame_pose = GpsPose(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                yaw=euler_from_quaternion([
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ])[2]
            )

            self._gps_transform.feedback_pose(map_frame_pose)
            self._gps_transform.add_utm_to_map_refpoint(
                self._gps_transform.last_utm, map_frame_pose)

            self.get_logger().info('feedback pose to gps')
            self.get_logger().info(
                f"after offset yaw: {self._gps_transform.origin_utm.yaw}")

            self.get_logger().info(f"len: {len(self._gps_transform._utm_to_map_list)}")
            self.get_logger().info("-----")

        # else:
        if True:
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

    def save_utm_refpoint_callback(self, request, response):
        if os.path.exists(self.utm_param_yaml_path):
            dir_path = os.path.dirname(self.utm_param_yaml_path)
            basename = os.path.basename(self.utm_param_yaml_path)
            filename, ext = os.path.splitext(basename)

            save_file_path = os.path.join(
                dir_path, filename + '_new' + ext)
        else:
            save_file_path = self.utm_param_yaml_path

        self._gps_transform.save_utm_refpoints(save_file_path)
        response.success = True
        response.message = f'saved utm refpoints to {self.utm_param_yaml_path}'
        return response


def main(args=None):
    rclpy.init(args=args)

    node = GpsTransformNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
