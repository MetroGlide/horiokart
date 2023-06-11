#!/usr/bin/env python3

import psutil

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


class PCResourcePublisherNode(Node):

    def __init__(self):
        super().__init__('pc_resource_publisher_node')
        self.get_logger().info('pc_resource_publisher_node started')

        self._publish_rate = self.declare_parameter(
            'publish_rate',
            1.0).value  # [Hz]
        self._cpu_interval = self.declare_parameter(
            'cpu_interval',
            0.5).value  # [sec]

        if self._cpu_interval > 1/self._publish_rate:
            self.get_logger().warn(
                'cpu_interval is larger than 1/publish_rate. '
                'cpu_interval is set to 1/publish_rate.')
            self._cpu_interval = 1/self._publish_rate

        self.cpu_usage_pub = self.create_publisher(
            Float32,
            'cpu_usage',
            1)
        self.mem_usage_pub = self.create_publisher(
            Float32,
            'memory_usage',
            1)

        self.timer = self.create_timer(
            1/self._publish_rate,
            self.timer_callback)

    def timer_callback(self):
        cpu_usage = psutil.cpu_percent(interval=self._cpu_interval)
        self.get_logger().debug(f'cpu_usage: {cpu_usage:.2f} %')
        self.cpu_usage_pub.publish(Float32(data=cpu_usage))

        mem_usage = psutil.virtual_memory().percent
        self.get_logger().debug(f'mem_usage: {mem_usage:.2f} %')
        self.mem_usage_pub.publish(Float32(data=mem_usage))


def main(args=None):
    rclpy.init(args=args)

    pc_resource_publisher_node = PCResourcePublisherNode()

    rclpy.spin(pc_resource_publisher_node)

    pc_resource_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
