#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter
from rcl_interfaces.srv import SetParameters
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from robot_localization.srv import SetPose

from tf_transformations import quaternion_from_euler, euler_from_quaternion


class InitialposeToSetpose(Node):
    def __init__(self):
        super().__init__('initialpose_to_setpose')
        self.get_logger().info('initialpose_to_setpose started')
        self.init_parameters()

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initialpose_callback,
            1)

        # set_pose service client
        self.set_pose_client = self.create_client(SetPose, 'set_pose')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pose service not available, waiting again...')
        self.get_logger().info('set_pose service available')

        # set_parameter service client
        self.set_parameter_client = self.create_client(SetParameters, '/ekf_global_node/set_parameters')
        while not self.set_parameter_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')
        self.get_logger().info('set_parameters service available')


    def init_parameters(self):
        self._pose = None

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self._pose = msg

        self.get_logger().info(f'initialpose_callback: {self._pose}')

        self.call_set_pose_service()
        # self.set_parameter_service_client()

    # set dynamic parameter init_state(double list[15]) to /ekf_localization_node/initial_state
    def set_parameter_service_client(self):
        req = SetParameters.Request()
        req.parameters = [Parameter()]

        x, y, z = self._pose.pose.pose.position.x, self._pose.pose.pose.position.y, self._pose.pose.pose.position.z
        roll, pitch, yaw = euler_from_quaternion([
            self._pose.pose.pose.orientation.x,
            self._pose.pose.pose.orientation.y,
            self._pose.pose.pose.orientation.z,
            self._pose.pose.pose.orientation.w
        ])

        req.parameters[0].name = 'initial_state'
        # req.parameters[0].value.type = Parameter.Type.DOUBLE_ARRAY
        req.parameters[0].value.double_array_value = [
            x, y, z,
            roll, pitch, yaw,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
        ]
        future = self.set_parameter_client.call_async(req)
        future.add_done_callback(self.set_parameter_response_callback)

    def set_parameter_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Set parameter from initialpose DONE')

    def call_set_pose_service(self):
        req:PoseWithCovarianceStamped = SetPose.Request()
        req.pose = self._pose
        future = self.set_pose_client.call_async(req)
        future.add_done_callback(self.set_pose_response_callback)

    def set_pose_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info(
                'Service call failed %r' % (e,))
        else:
            self.get_logger().info('Set pose from initialpose DONE')


def main(args=None):
    rclpy.init(args=args)

    initialpose_to_setpose = InitialposeToSetpose()

    rclpy.spin(initialpose_to_setpose)

    initialpose_to_setpose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
