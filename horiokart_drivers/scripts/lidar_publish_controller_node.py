#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class LidarPublishControllerNode(Node):
    def __init__(self):
        super().__init__("lidar_publish_controller_node")

        self.get_logger().info("lidar_publish_controller_node started")
        self._publish = False

        self.init_parameters()
        self.init_ros_topics()

        self.get_logger().info("lidar_publish_controller_node initialized")

    def init_parameters(self):
        pass

    def init_ros_topics(self):
        self._subscriber = self.create_subscription(
            LaserScan,
            "scan_origin",
            self.scan_callback,
            1
        )

        self._publisher = self.create_publisher(
            LaserScan,
            "scan",
            1
        )

        self._change_publish_state_service = self.create_service(
            SetBool,
            "~/change_publish_state",
            self.change_publish_state_callback
        )

    def scan_callback(self, msg: LaserScan):
        if self._publish:
            self._publisher.publish(msg)

    def change_publish_state_callback(self, req, res):
        self._publish = req.data
        res.success = True
        res.message = "success"
        return res


if __name__ == "__main__":
    rclpy.init()
    node = LidarPublishControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
