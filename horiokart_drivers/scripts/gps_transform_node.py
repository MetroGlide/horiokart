#!/usr/bin/env python3

import os
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool

from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import tf2_py
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from gps_transform import GpsTransform
from gps_transform import Pose as GpsPose


class HDOP2Covariance:
    def __init__(self):
        self.a = 19.23076923
        self.b = -4.230769231

        self.mean_num = 10
        self.covariance_history = []

    def get_covariance(self, hdop: float) -> float:
        cov = (hdop * self.a + self.b) ** 2
        self.covariance_history.append(cov)

        if len(self.covariance_history) > self.mean_num:
            self.covariance_history.pop(0)

        return float(sum(self.covariance_history) / len(self.covariance_history))


class GpsTransformNode(Node):
    def __init__(self):
        super().__init__('gps_transform_node')
        self.get_logger().info('gps_transform_node started')

        self.init_parameters()
        self.prepare_tf()
        self.init_ros_communication()

        self._hdop_to_covariance = HDOP2Covariance()

    def init_parameters(self):
        self.resistration = self.declare_parameter(
            'resistration',
            # True).value
            False).value

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
            '/root/ros2_data/map/utm_to_map_refpoint.yaml').value

        self.covariance_publish_threshold = self.declare_parameter(
            'covariance_publish_threshold',
            10.0).value

        self._gps_transform = GpsTransform(offset_yaw=2.75)
        if not self._gps_transform.load_utm_to_map_list(self.utm_param_yaml_path):
            self.get_logger().info(
                f'failed to load {self.utm_param_yaml_path}')

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

        # for debug
        self.odom_gps_ref_pub = self.create_publisher(
            Odometry,
            'odom/gps/ref_point',
            1)
        self.odom_gps_ref_pub

        self.current_odom_msg = None
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            1)

        self.set_resistration_mode_srv = self.create_service(
            SetBool,
            '~/set_resistration_mode',
            self.set_resistration_mode_callback)

        self.save_utm_refpoints_srv = self.create_service(
            Trigger,
            '~/save_utm_refpoints',
            self.save_utm_refpoint_callback)

        self.marker_array_publisher = self.create_publisher(
            MarkerArray,
            '~/utm_to_map_refpoint_markers',
            1)

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
            self.get_logger().warn(
                f"tf not found. between {self.robot_base_frame_id} and {self.gps_frame_id}")
            return

        self._gps_to_base_link_transform = tf2_py.inverse_transform(
            self._base_link_to_gps_transform)

    def hdop_to_covariance(self, hdop: float) -> float:
        a = 19.23076923
        b = -4.230769231
        return (hdop * a + b) ** 2  # temporary
        # return a * math.exp(-b * hdop)

    def gps_callback(self, msg):
        map_position = self._gps_transform.transform_to_map(
            msg.latitude, msg.longitude)

        covariance = self._hdop_to_covariance.get_covariance(
            msg.position_covariance[0])

        # listen tf from map to base_link
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame_id,  # target frame
                self.gps_frame_id,  # source frame
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f'tf not found. between {self.map_frame_id} and {self.gps_frame_id}')
            return

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
        if self.resistration:
            # TODO!: for debug
            if self._gps_transform.add_utm_to_map_refpoint(self._gps_transform.last_utm, map_frame_pose, covariance):
                pass

        # if len(self._gps_transform._utm_to_map_converter.utm_to_map_list) < 2:
        #     return
        # if len(self._gps_transform._utm_to_map_converter.utm_to_map_list) > 3 and self.resistration:
        #     self.resistration = False

        else:
        # if True:  # TODO!: for debug
            if covariance > self.covariance_publish_threshold:
                return

            self.odom_msg.header.stamp = msg.header.stamp
            self.odom_msg.pose.pose.position.x = map_position.x
            self.odom_msg.pose.pose.position.y = map_position.y
            self.odom_msg.pose.pose.position.z = 0.0
            # set orientation from yaw
            q = quaternion_from_euler(0.0, 0.0, map_position.yaw)
            self.odom_msg.pose.pose.orientation = Quaternion(
                x=q[0], y=q[1], z=q[2], w=q[3])

            # set covariance
            self.odom_msg.pose.covariance[0] = covariance
            self.odom_msg.pose.covariance[7] = covariance
            # self.odom_msg.pose.covariance[14] = 0.0
            self.odom_msg.pose.covariance[14] = 1.5
            self.odom_msg.pose.covariance[21] = 0.0
            self.odom_msg.pose.covariance[28] = 0.0
            self.odom_msg.pose.covariance[35] = 0.0

            self.odom_pub.publish(self.odom_msg)

        self._publish_ref_points()

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

        self._gps_transform.save_utm_to_map_list(save_file_path)
        response.success = True
        response.message = f'saved utm refpoints to {self.utm_param_yaml_path}'
        return response

    def _create_marker(self, ref_point: GpsPose, index: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "utm_to_map_refpoint"
        marker.id = index
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = ref_point.map.x
        marker.pose.position.y = ref_point.map.y
        marker.pose.position.z = 0.0
        # set orientation from yaw
        q = quaternion_from_euler(0.0, 0.0, ref_point.map.yaw)
        marker.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        return marker

    def _publish_ref_points(self):
        marker_array = MarkerArray()
        for index, ref_point in enumerate(self._gps_transform._utm_to_map_converter.utm_to_map_list):
            marker = self._create_marker(ref_point, index)
            marker_array.markers.append(marker)
        self.marker_array_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = GpsTransformNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
