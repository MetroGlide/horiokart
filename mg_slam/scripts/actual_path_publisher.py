#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf2_ros import TransformListener, Buffer


class ActualPathPublisher(Node):
    def __init__(self):
        super().__init__('actual_path_publisher')
        self.get_logger().info('actual_path_publisher started')
        self.init_parameters()

        self.path_pub = self.create_publisher(
            Path,
            'actual_path',
            1)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # prepare tf2 listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._path_list = []

        # create timer
        self.create_timer(self._rate, self.publish_path)

    def init_parameters(self):
        self._rate = self.declare_parameter('rate', 1.0).value
        self._path_length = self.declare_parameter('path_length', 100).value
        self._frame_id = self.declare_parameter('frame_id', 'map').value
        self._child_frame_id = self.declare_parameter(
            'child_frame_id', 'base_link').value

    def append_path(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._child_frame_id,
                rclpy.time.Time())
        except Exception as e:
            self.get_logger().info('tf not found')
            return

        self._path_list.append(
            PoseStamped(
                header=transform.header,
                pose=Pose(
                    position=Point(
                        x=transform.transform.translation.x,
                        y=transform.transform.translation.y,
                        z=transform.transform.translation.z
                    ),
                    orientation=Quaternion(
                        x=transform.transform.rotation.x,
                        y=transform.transform.rotation.y,
                        z=transform.transform.rotation.z,
                        w=transform.transform.rotation.w
                    )
                )
            )
        )

        if len(self._path_list) > self._path_length:
            self._path_list.pop(0)

    def publish_path(self):
        self.append_path()

        self.path_msg.poses = self._path_list
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActualPathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
