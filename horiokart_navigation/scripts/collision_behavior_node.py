#!/usr/bin/env python3
import os
import time
import enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.msg import CollisionDetectorState
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from std_srvs.srv import Trigger

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class CollisionBehavior(Node):
    """
    Check front polygon collision flag
    emergency stop
    waypoint follower stop
    wait for specified time
    back up specified distance while rear polygon collision flag is false
    """

    """
    Expected collision dector state topic
    polygons: [front, rear]
    """

    class Phase(enum.Enum):
        UNKNOWN = enum.auto()
        NO_COLLISION = enum.auto()
        WAIT = enum.auto()
        BACK_UP = enum.auto()
        FINISHED = enum.auto()

    def __init__(self):
        super().__init__('collision_behavior')

        self._init_ros_parameters()
        self._init_ros_communications()

        self._phase = self.Phase.UNKNOWN
        self._same_detection_flag = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._emergency_stop = Bool()
        self._emergency_stop.data = False

        self._emergency_stop_pub.publish(self._emergency_stop)

        self._controller_timer = self.create_timer(
            1.0 / self._controller_rate,
            self._controller_main
        )

        self.waypoint_follower_stop_future = None
        while not self._waypoint_follower_stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'Waypoint follower stop service not available, waiting again...'
            )

        self.get_logger().info(
            'Collision behavior initialized.'
        )

    def _init_ros_parameters(self):
        self._stop_wait_time = self.declare_parameter(
            'stop_wait_time',
            5.0
        ).value
        self._back_up_distance = self.declare_parameter(
            'back_up_distance',
            1.0  # m
        ).value
        self._back_up_velocity = self.declare_parameter(
            'back_up_velocity',
            0.2  # m/s
        ).value
        self._controller_rate = self.declare_parameter(
            'controller_rate',
            10.0
        ).value
        self._retry_max_count = self.declare_parameter(
            'retry_max_count',
            3
        ).value
        self._retry_count = 0

    def _init_ros_communications(self):
        self._collision_detector_sub = self.create_subscription(
            CollisionDetectorState,
            '/collision_detector_state',
            self._collision_detector_subscriber_callback,
            1
        )

        self._emergency_stop_pub = self.create_publisher(
            Bool,
            '/motor_driver_node/emergency_stop',
            1
        )

        self._twist_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self._waypoint_follower_stop_client = self.create_client(
            Trigger,
            '/waypoint_follower_node/stop'
        )
        self._waypoint_follower_start_client = self.create_client(
            Trigger,
            '/waypoint_follower_node/start'
        )

    def _collision_detector_subscriber_callback(self, msg: CollisionDetectorState):
        if msg.detections[0]:
            if self._same_detection_flag:
                return

            self._same_detection_flag = True

            if self._phase == self.Phase.NO_COLLISION or self._phase == self.Phase.UNKNOWN:
                self._emergency_stop.data = True
                self._emergency_stop_pub.publish(self._emergency_stop)

                self.get_logger().info(
                    'Collision detected. Emergency stop.'
                )

                if self._waypoint_follower_stop_client.wait_for_service(timeout_sec=1.0):
                    waypoint_follower_stop_request = Trigger.Request()
                    self.waypoint_follower_stop_future = self._waypoint_follower_stop_client.call_async(
                        waypoint_follower_stop_request
                    )

                else:
                    self.get_logger().error(
                        'Failed to call waypoint follower stop service.'
                    )
                    self._reset_emergency()

        else:
            self._same_detection_flag = False

            if self._phase == self.Phase.UNKNOWN:
                self._phase = self.Phase.NO_COLLISION

            if not self._phase in [self.Phase.UNKNOWN, self.Phase.NO_COLLISION, self.Phase.FINISHED]:
                self._collision_resolved()

                self.get_logger().info(
                    'Collision resolved. Emergency stop released.'
                )

        self._latest_msg = msg

    def _reset_emergency(self):
        # send emergency stop false
        self._emergency_stop.data = False
        self._emergency_stop_pub.publish(self._emergency_stop)

    def _collision_resolved(self):
        self._phase = self.Phase.NO_COLLISION
        self._reset_emergency()

        if self._waypoint_follower_start_client.wait_for_service(timeout_sec=1.0):
            waypoint_follower_start_request = Trigger.Request()
            waypoint_follower_start_future = self._waypoint_follower_start_client.call_async(
                waypoint_follower_start_request
            )
        else:
            self.get_logger().error(
                'Failed to call waypoint follower start service.'
            )

    def _check_stop_response_done(self):
        if self.waypoint_follower_stop_future is None:
            return
        if not self.waypoint_follower_stop_future.done():
            self.get_logger().info(
                'Waiting for waypoint follower stop service response...'
            )

        if self.waypoint_follower_stop_future.result().success:
            self.get_logger().info(
                'Waypoint follower stop service succeeded.'
            )

            # initialize phase
            self._phase = self.Phase.WAIT
            self._wait_start_time = None
            self._start_position = None
            self._retry_count = 0

        else:
            self.get_logger().error(
                'Waypoint follower stop service failed.'
            )
            self._reset_emergency()

        self.waypoint_follower_stop_future = None

    def _controller_main(self):
        self._check_stop_response_done()

        if self._phase in [self.Phase.UNKNOWN, self.Phase.NO_COLLISION]:
            return

        # check collition resolved
        if not self._latest_msg.detections[0]:
            self._emergency_stop.data = False
            self._emergency_stop_pub.publish(self._emergency_stop)

            self.get_logger().info(
                'Collision resolved. Emergency stop released.'
            )

            self._phase = self.Phase.NO_COLLISION
            return

        if self._phase == self.Phase.WAIT:
            # initialize
            if self._wait_start_time is None:
                self._wait_start_time = time.time()
                self.get_logger().info(
                    f"Wait for {self._stop_wait_time} seconds."
                )
                return

            if time.time() - self._wait_start_time > self._stop_wait_time:
                self._phase = self.Phase.BACK_UP
                self._reset_emergency()
                self._wait_start_time = None
                return

            return

        if self._phase == self.Phase.BACK_UP:
            # initialize
            if self._start_position is None:
                self._start_position = self._tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                ).transform.translation

                self.get_logger().info(
                    f"Start back up. velocity: {self._back_up_velocity}, distance: {self._back_up_distance}"
                )

                return

            # check rear collision
            if self._latest_msg.detections[1]:
                # publish 0 velocity
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self._twist_pub.publish(twist)

                self.get_logger().warn(
                    'Collision detected at rear. Back up.'
                )

                self._retry_count += 1
                if self._retry_count > self._retry_max_count:
                    self.get_logger().error(
                        'Collision detected at rear. Retry count exceeded. Collision behavior failed.'
                    )
                    self._phase = self.Phase.NO_COLLISION
                    self._retry_count = 0

                    self._reset_emergency()
                    return

                return

            # send back up velocity
            twist = Twist()
            twist.linear.x = -1 * abs(self._back_up_velocity)
            twist.angular.z = 0.0
            self._twist_pub.publish(twist)

            # check back up distance
            current_position = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            ).transform.translation

            distance = ((current_position.x - self._start_position.x) **
                        2 + (current_position.y - self._start_position.y) ** 2) ** 0.5

            if distance > self._back_up_distance:
                self.get_logger().info(
                    'Back up finished.'
                )
                self._retry_count = 0

                self._phase = self.Phase.FINISHED
                return

            return

        if self._phase == self.Phase.FINISHED:
            self._collision_resolved()
            self._phase = self.Phase.NO_COLLISION
            self.get_logger().info(
                'Collision behavior finished.'
            )
            return


def main(args=None):
    rclpy.init(args=args)

    collision_behavior = CollisionBehavior()

    rclpy.spin(collision_behavior)

    collision_behavior.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
