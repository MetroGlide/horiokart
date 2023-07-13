#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from horiokart_navigation.waypoint import WaypointList, Waypoint, get_index_from_waypoint_name, WaypointsLoader, WaypointsSaver


class WaypointEditorNode(Node):
    def __init__(self):
        super().__init__('waypoint_editor_node')

        self.waypoints = WaypointList()
        self._interactive_marker_server = InteractiveMarkerServer(
            self, 'waypoint_editor')

        self._save_path = self.declare_parameter('save_path').value
        load_path = self.declare_parameter('load_path').value
        if load_path != '':
            self.load_waypoints_from_file(load_path)

        self._pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._pose_callback,
            1)

        self._save_service = self.create_service(
            Trigger,
            'save_waypoints',
            self._save_waypoints_callback)
        self._force_save_service = self.create_service(
            Trigger,
            'force_save_waypoints',
            self._force_save_waypoints_callback)

    def _normalize_quaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
            quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

        return quaternion_msg

    def _create_interactive_marker(self, index: int, pose_stamped: PoseStamped):
        pose_stamped.pose.position.z += 1.0

        # Create InteractiveMarker
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = 'map'
        interactive_marker.name = f"waypoint_{index}"
        interactive_marker.description = f"waypoint_{index}"
        interactive_marker.pose = pose_stamped.pose

        marker = Marker()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 0.45
        marker.scale.z = 0.45
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Create a control for the InteractiveMarker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)

        interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.orientation = self._normalize_quaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.orientation = self._normalize_quaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        interactive_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.orientation = self._normalize_quaternion(control.orientation)
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        interactive_marker.controls.append(control)

        self._interactive_marker_server.insert(
            interactive_marker, feedback_callback=self._interactive_marker_feedback_callback)

        self._interactive_marker_server.applyChanges()
        self.get_logger().info(
            f"InteractiveMarker {interactive_marker.name} created.")

    def _interactive_marker_feedback_callback(self, feedback: InteractiveMarkerFeedback):

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            # self.waypoints.update(
            #     get_index_from_waypoint_name(feedback.marker_name), feedback.pose)
            pass

        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.get_logger().info(
                f"Waypoint {feedback.marker_name} updated.")

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = feedback.pose

            self.waypoints.update_pose(get_index_from_waypoint_name(
                feedback.marker_name), pose_stamped)

            self._interactive_marker_server.applyChanges()

    def _save_waypoints_callback(self, request, response):
        file_path = self._save_path

        if os.path.exists(file_path):
            response.success = False
            response.message = f"File {file_path} already exists."
            self.get_logger().error(response.message)
            return response

        self.save_waypoints_to_file(file_path)

        response.success = True
        response.message = 'Waypoints saved to {}'.format(file_path)

        self.get_logger().info(response.message)
        return response

    def _force_save_waypoints_callback(self, request, response):
        file_path = self._save_path

        self.save_waypoints_to_file(file_path)

        response.success = True
        response.message = f'Waypoints force saved to {file_path}'

        self.get_logger().info(response.message)
        return response

    def save_waypoints_to_file(self, file_path):
        saver = WaypointsSaver(file_path)
        saver.save(self.waypoints)

    def load_waypoints_from_file(self, file_path):
        loader = WaypointsLoader(file_path)
        self.waypoints = loader.load()

        for i, waypoint in enumerate(self.waypoints.get_all()):
            self._create_interactive_marker(
                i, waypoint.pose)

    def _pose_callback(self, msg: PoseStamped):
        self._create_interactive_marker(
            self.waypoints.get_next_index(), msg)
        self.waypoints.add(
            Waypoint(
                index=self.waypoints.get_next_index(),
                pose=msg,
                reach_tolerance=0.5,
                on_reached_action=[]
            )
        )


def main(args=None):
    rclpy.init(args=args)
    waypoint_editor_node = WaypointEditorNode()

    rclpy.spin(waypoint_editor_node)
    waypoint_editor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
