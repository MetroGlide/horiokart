#!/usr/bin/env python3
import os
import copy
import pprint
import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler

from horiokart_navigation.waypoint import WaypointList, Waypoint, get_index_from_waypoint_name, WaypointsLoader, WaypointsSaver


class InteractiveWaypointMarker:
    def __init__(self, waypoint: Waypoint):
        self.waypoint = waypoint
        self._create_interactive_marker()

    def _create_interactive_marker(self):
        pose_stamped = self.waypoint.pose
        pose_stamped.pose.position.z += 1.0

        index = self.waypoint.index

        # Create InteractiveMarker
        interactive_marker = InteractiveMarker()

        interactive_marker.header.frame_id = 'map'
        interactive_marker.pose = pose_stamped.pose

        marker = Marker()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.pose.position.y = -1.0
        text_marker.pose.position.x = -1.0
        text_marker.action = Marker.ADD
        text_marker.scale.x = 2.0
        text_marker.scale.y = 2.0
        text_marker.scale.z = 0.45
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0

        detail_text_marker = Marker()
        detail_text_marker.type = Marker.TEXT_VIEW_FACING
        detail_text_marker.pose.position.y = -2.0
        detail_text_marker.pose.position.x = -0.0
        detail_text_marker.action = Marker.ADD
        detail_text_marker.scale.x = 0.1
        detail_text_marker.scale.y = 0.1
        detail_text_marker.scale.z = 0.1
        detail_text_marker.color.a = 1.0
        detail_text_marker.color.r = 0.7
        detail_text_marker.color.g = 0.3
        detail_text_marker.color.b = 0.0

        detail_text_marker.text = f"{pprint.pformat(self.waypoint.to_dict(), width=15).replace(' ', '')}"
        # detail_text_marker.text = f"{self.waypoint.to_dict()}"

        # Create a control for the InteractiveMarker
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(marker)
        control.markers.append(text_marker)
        control.markers.append(detail_text_marker)

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

        self.interactive_marker = interactive_marker
        self.text_marker = text_marker
        self.detail_text_marker = detail_text_marker

        self.set_index(index)
        self.set_color_from_waypoint_type()

    def set_index(self, index: int):
        self.waypoint.index = index
        self.interactive_marker.name = f"waypoint_{index}"
        self.interactive_marker.description = f"waypoint_{index}"

        self.text_marker.text = f"No.{index}"

    def set_color(self, r: float, g: float, b: float):
        self.interactive_marker.controls[0].markers[0].color.r = r
        self.interactive_marker.controls[0].markers[0].color.g = g
        self.interactive_marker.controls[0].markers[0].color.b = b

    def set_color_from_waypoint_type(self):
        if self.waypoint.is_through_point:
            self.set_color(0.0, 1.0, 0.0)
        else:
            self.set_color(1.0, 0.0, 0.0)

    def _normalize_quaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
            quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

        return quaternion_msg

    def get_interactive_marker(self):
        self._create_interactive_marker()
        return self.interactive_marker

    def update_pose(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # self.get_logger().info(
            #     f"Waypoint {feedback.marker_name} updated.")

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose = feedback.pose

            self.waypoint.pose = pose_stamped
            print(f"Waypoint {feedback.marker_name} updated.")
            print(f"Pose: {feedback.pose}")


class InteractiveWaypointsManager:
    def __init__(self, interactive_marker_server: InteractiveMarkerServer):
        self.i_waypoints = []
        self.interactive_marker_server = interactive_marker_server
        self.menu_handler = MenuHandler()

        self.init_menu()

    def _insert_server(self, interactive_waypoint: InteractiveWaypointMarker):
        self.interactive_marker_server.insert(
            interactive_waypoint.get_interactive_marker(),
            feedback_callback=interactive_waypoint.update_pose
        )
        self.menu_handler.apply(self.interactive_marker_server,
                                interactive_waypoint.get_interactive_marker().name)
        self.interactive_marker_server.applyChanges()

    def add(self, interactive_waypoint: InteractiveWaypointMarker):
        self.i_waypoints.append(interactive_waypoint)
        self._insert_server(interactive_waypoint)

    def add_default(self, pose_stamped: PoseStamped):
        i_waypoint = InteractiveWaypointMarker(
            Waypoint(
                index=self.get_next_index(),
                pose=pose_stamped,
                reach_tolerance=1.0,
                on_reached_action=[]
            )
        )
        self.add(i_waypoint)

    def remove(self, index: int):
        self.i_waypoints.pop(index)
        self.reindex()

        self.refresh()

    def insert(self, index: int, interactive_waypoint: InteractiveWaypointMarker):
        self.insert_index(index)
        self.i_waypoints.insert(index, interactive_waypoint)

        self.refresh()

    def refresh(self):
        self.interactive_marker_server.clear()
        for i_waypoint in self.i_waypoints:
            self._insert_server(i_waypoint)

    def get(self, index: int) -> InteractiveWaypointMarker:
        return self.i_waypoints[index]

    def get_all(self) -> list:
        return self.i_waypoints

    def get_size(self) -> int:
        return len(self.i_waypoints)

    def clear(self):
        self.i_waypoints = []

    def get_next_index(self) -> int:
        return self.get_size()

    def sort_by_index(self):
        self.i_waypoints.sort(key=lambda i_waypoint: i_waypoint.waypoint.index)

    def reindex(self):
        self.sort_by_index()
        for i, i_waypoint in enumerate(self.i_waypoints):
            i_waypoint.set_index(i)

    def insert_index(self, index: int):
        self.sort_by_index()
        for i_waypoint in self.i_waypoints:
            if i_waypoint.waypoint.index >= index:
                i_waypoint.set_index(i_waypoint.waypoint.index + 1)

    def convert_waypoint_list(self) -> WaypointList:
        waypoint_list = WaypointList()
        for i_waypoint in self.i_waypoints:
            waypoint_list.add(i_waypoint.waypoint)

        return waypoint_list

    @staticmethod
    def from_waypoint_list(waypoint_list: WaypointList, interactive_marker_server: InteractiveMarkerServer):
        i_waypoints = InteractiveWaypointsManager(
            interactive_marker_server=interactive_marker_server)
        for waypoint in waypoint_list.get_all():
            i_waypoints.add(InteractiveWaypointMarker(waypoint))

        return i_waypoints

    # Menu definition
    def init_menu(self):
        self.menu_handler.insert(
            "Copy Next", callback=self._copy_next_waypoint_callback)
        self.menu_handler.insert(
            "Copy Next(Pose only)", callback=self._copy_next_pose_only_waypoint_callback)
        self.menu_handler.insert(
            "~~Approach~~", callback=self._approach_waypoint_callback)
        self.menu_handler.insert(
            "Delete", callback=self._delete_waypoint_callback)

        self.menu_handler.insert(
            "-----", callback=self._dummy_callback)

        self.menu_handler.insert(
            "Set through point", callback=self._set_through_point_callback)
        self.menu_handler.insert(
            "Set target point", callback=self._set_target_point_callback)

    def _dummy_callback(self, feedback):
        pass

    def _copy_next_waypoint_callback(self, feedback):
        index = get_index_from_waypoint_name(feedback.marker_name)
        i_waypoint = copy.deepcopy(self.get(index))
        i_waypoint.set_index(index + 1)
        self.insert(index + 1, i_waypoint)

    def _copy_next_pose_only_waypoint_callback(self, feedback):
        index = get_index_from_waypoint_name(feedback.marker_name)
        i_waypoint = InteractiveWaypointMarker(
            Waypoint(
                index=index + 1,
                pose=self.get(index).waypoint.pose,
                reach_tolerance=1.0,
                on_reached_action=[]
            )
        )
        self.insert(index + 1, i_waypoint)

    def _approach_waypoint_callback(self, feedback):
        pass

    def _delete_waypoint_callback(self, feedback):
        index = get_index_from_waypoint_name(feedback.marker_name)
        self.remove(index)

    def _set_through_point_callback(self, feedback):
        index = get_index_from_waypoint_name(feedback.marker_name)
        self.i_waypoints[index].waypoint.is_through_point = True
        self.refresh()

    def _set_target_point_callback(self, feedback):
        index = get_index_from_waypoint_name(feedback.marker_name)
        self.i_waypoints[index].waypoint.is_through_point = False
        self.refresh()


class WaypointEditorNode(Node):
    def __init__(self):
        super().__init__('waypoint_editor_node')

        self._interactive_marker_server = InteractiveMarkerServer(
            self, 'waypoint_editor')

        self._interactive_waypoints_manager = InteractiveWaypointsManager(
            self._interactive_marker_server)

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

    def _save_waypoints_callback(self, request, response):
        file_path = self._save_path

        if os.path.exists(file_path):
            response.success = False
            response.message = f"File {file_path} already exists."
            self.get_logger().error(response.message)
            return response

        file_path = self.save_waypoints_to_file(file_path)

        response.success = True
        response.message = 'Waypoints saved to {}'.format(file_path)

        self.get_logger().info(response.message)
        return response

    def _force_save_waypoints_callback(self, request, response):
        file_path = self._save_path

        file_path = self.save_waypoints_to_file(file_path)

        response.success = True
        response.message = f'Waypoints force saved to {file_path}'

        self.get_logger().info(response.message)
        return response

    def save_waypoints_to_file(self, file_path):
        if not os.path.exists(os.path.dirname(file_path)):
            os.makedirs(os.path.dirname(file_path))
        if os.path.exists(file_path):
            # add date to enc of file without extension
            filename, ext = os.path.splitext(file_path)
            file_path = filename + \
                f"_{datetime.datetime.now().strftime('%Y%m%d-%H%M%S')}" + ext

        saver = WaypointsSaver(file_path)
        saver.save(self._interactive_waypoints_manager.convert_waypoint_list())

        return file_path

    def load_waypoints_from_file(self, file_path):
        loader = WaypointsLoader(file_path)
        self._interactive_waypoints_manager = InteractiveWaypointsManager.from_waypoint_list(
            loader.load(),
            self._interactive_marker_server)

    def _pose_callback(self, msg: PoseStamped):
        self._interactive_waypoints_manager.add_default(msg)


def main(args=None):
    rclpy.init(args=args)
    waypoint_editor_node = WaypointEditorNode()

    rclpy.spin(waypoint_editor_node)
    waypoint_editor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
