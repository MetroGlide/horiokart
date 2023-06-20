#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
import yaml

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarkerPose
from visualization_msgs.msg import InteractiveMarkerUpdate
from visualization_msgs.srv import GetInteractiveMarkers
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from dataclasses import dataclass


@dataclass
class Waypoint:
    index: int
    pose: PoseStamped

    def to_dict(self):
        return {
            'index': self.index,
            'pose': {
                'position': {
                    'x': self.pose.position.x,
                    'y': self.pose.position.y,
                    'z': self.pose.position.z
                },
                'orientation': {
                    'x': self.pose.orientation.x,
                    'y': self.pose.orientation.y,
                    'z': self.pose.orientation.z,
                    'w': self.pose.orientation.w
                }
            }
        }


class WaypointList:
    def __init__(self):
        self.waypoints = []

    def add(self, waypoint: PoseStamped):
        self.waypoints.append(
            Waypoint(
                index=self.get_next_index(),
                pose=waypoint
            )
        )

    def remove(self, index: int):
        self.waypoints.pop(index)

    def update(self, index: int, pose: PoseStamped):
        self.waypoints[index].pose = pose

    def get(self, index: int) -> Waypoint:
        return self.waypoints[index]

    def get_all(self) -> list:
        return self.waypoints

    def get_size(self) -> int:
        return len(self.waypoints)

    def clear(self):
        self.waypoints = []

    def get_next_index(self) -> int:
        return self.get_size()


def get_index_from_waypoint_name(waypoint_name):
    return int(waypoint_name.split('_')[1])


class WaypointEditorNode(Node):
    def __init__(self):
        super().__init__('waypoint_editor_node')

        self.waypoints = WaypointList()

        # self._load_file_path = self.declare_parameter('load_file_path', '/root/ros2_data/waypoints.yaml').value
        _load_file_path = self.declare_parameter('load_file_path', '').value
        if _load_file_path != '':
            self.load_waypoints_from_file(_load_file_path)

        self._pose_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._pose_callback,
            1)

        self._interactive_marker_server = InteractiveMarkerServer(
            self, 'waypoint_editor')

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
        # marker.header.frame_id = 'map'
        marker.type = Marker.CUBE
        # marker.action = Marker.ADD
        marker.scale.x = 0.45
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

            self.waypoints.update(get_index_from_waypoint_name(
                feedback.marker_name), pose_stamped)

            self._interactive_marker_server.applyChanges()

    def save_waypoints_callback(self, request, response):
        file_path = '/app/waypoints.yaml'  # Specify the YAML file path here
        self.save_waypoints_to_file(file_path)
        response.success = True
        response.message = 'Waypoints saved to {}'.format(file_path)
        return response

    def save_waypoints_to_file(self, file_path):
        waypoints = []
        for waypoint in self.waypoints.get_all():
            waypoints.append(waypoint.to_dict())

        with open(file_path, 'w') as file:
            yaml.dump(waypoints, file)

    def load_waypoints_from_file(self, file_path):
        with open(file_path, 'r') as file:
            waypoints = yaml.load(file, Loader=yaml.FullLoader)

        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position = Point(*waypoint['position'])
            pose.pose.orientation = Quaternion(*waypoint['orientation'])
            self.waypoints.add(pose)

    def _pose_callback(self, msg):
        self._create_interactive_marker(
            self.waypoints.get_next_index(), msg)
        self.waypoints.add(msg)


def main(args=None):
    rclpy.init(args=args)
    waypoint_editor_node = WaypointEditorNode()

    rclpy.spin(waypoint_editor_node)
    waypoint_editor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
