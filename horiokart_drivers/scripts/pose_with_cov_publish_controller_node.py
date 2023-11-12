#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool


class PoseWithCovariancePublishControllerNode(Node):
    def __init__(self):
        super().__init__("pose_with_covariance_publish_controller_node")

        self.get_logger().info("pose_with_covariance_publish_controller_node started")
        self._publish = True

        self.init_parameters()
        self.init_ros_topics()

        self.get_logger().info("pose_with_covariance_publish_controller_node initialized")

    def init_parameters(self):
        pass

    def init_ros_topics(self):
        self._subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            "pose_with_cov_origin",
            self.pose_callback,
            1
        )

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "pose_with_cov",
            1
        )

        self._change_publish_state_service = self.create_service(
            SetBool,
            "~/change_publish_state",
            self.change_publish_state_callback
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        if self._publish:
            self._publisher.publish(msg)

    def change_publish_state_callback(self, req, res):
        self._publish = req.data
        res.success = True
        res.message = "success"
        return res


if __name__ == "__main__":
    rclpy.init()
    node = PoseWithCovariancePublishControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
