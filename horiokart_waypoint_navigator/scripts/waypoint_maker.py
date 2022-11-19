#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerServer, InteractiveMarkerFeedback
from geometry_msgs.msg import PoseStamped
from horiokart_waypoint_msgs.srv import SaveWaypoint, SaveWaypointRequest, SaveWaypointResponse

from waypoint_navigator import Waypoint, WaypointLoaderCSV, WaypointLoaderBase

import os
import csv
from typing import List, Dict
import copy


class WaypointManager():
    def __init__(self) -> None:
        self._waypoint_dict: Dict[int, Waypoint] = {}

    def get_next_index(self) -> int:
        return self._get_max_index() + 1

    def update_pose(self, index: int, pose: PoseStamped):
        self._waypoint_dict[index].pose = pose

    def append_waypoint(self, waypoint: Waypoint):
        k = waypoint.index
        if k in self._waypoint_dict.keys():
            rospy.logerr(f"Aleady exist index {k}")
            return
        self._waypoint_dict[waypoint.index] = waypoint
        rospy.loginfo(f"Append waypoint index {k}")

    def _get_max_index(self):
        return max(self._waypoint_dict.keys()) if len(self._waypoint_dict.keys()) > 0 else -1

    def get_all_waypoints(self) -> List[Waypoint]:
        return self._waypoint_dict.values()


class WaypointMaker():
    def __init__(self, path: str, waypoint_loader: WaypointLoaderCSV = None) -> None:
        self._path = path
        # self._marker_list: List[Marker] = []

        self._waypoint_manager = WaypointManager()

        # self._waypoints_pub = rospy.Publisher(
        #     "waypoint", MarkerArray, queue_size=1)
        self._posestamp_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self._pose_sub_cb)

        self._save_srv = rospy.Service(
            "~save",
            SaveWaypoint,
            self._save_srv_cb
        )

        self._server = InteractiveMarkerServer("waypoint_server")

        if waypoint_loader is not None:
            self._load(waypoint_loader=waypoint_loader)
            self._server.applyChanges()

    def _load(self, waypoint_loader: WaypointLoaderCSV):
        rospy.loginfo(f"Load waypoint")
        for w in waypoint_loader.get_all_waypoints():
            self._waypoint_manager.append_waypoint(waypoint=w)
            self._make_marker(w.index, w.pose)

    def _pose_sub_cb(self, data: PoseStamped):
        # self._marker_list.append(self._make_marker(pos))
        index = self._waypoint_manager.get_next_index()
        self._make_marker(index, data)
        self._waypoint_manager.append_waypoint(
            Waypoint(
                index=index,
                pose=data,
                flag=0,
                reach_threshold=2.0,  # todo
                is_stop=False
            )
        )

        pos = data.pose
        print(
            f"{pos.position.x},{pos.position.y},0.0,0.0,0.0,{pos.orientation.z},{pos.orientation.w},")

    def _interactive_feedback_cb(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(f"update index {feedback.marker_name}")

            pose_stamped = PoseStamped()
            pose_stamped.pose = feedback.pose
            self._waypoint_manager.update_pose(
                index=int(feedback.marker_name),
                pose=pose_stamped
            )
            self._server.applyChanges()

    def _save_srv_cb(self, req: SaveWaypointRequest) -> SaveWaypointResponse:
        path = os.path.join(self._path, req.file_name)
        with open(path, 'w') as f:
            writer = csv.writer(f)
            # for p in self._marker_list:
            # pos = p.pose.position
            # rot = p.pose.orientation
            # writer.writerow(
            #     [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w]
            # )
            for waypoint in self._waypoint_manager.get_all_waypoints():
                pos = waypoint.pose.pose.position
                rot = waypoint.pose.pose.orientation
                writer.writerow(
                    [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w,  # 0~6
                     waypoint.reach_threshold,  # 7
                     int(waypoint.is_stop)  # 8
                     ]
                )
        res = SaveWaypointResponse()
        res.success = True
        res.path = path
        return res

    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
            quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def _make_marker(self, index: int, pose_stamped: PoseStamped):

        pose_stamped.pose.position.z += 1
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = pose_stamped.pose
        int_marker.name = str(index)
        int_marker.description = str(index)

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)

        int_marker.controls.append(box_control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        self.normalizeQuaternion(control.orientation)
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        self.normalizeQuaternion(control.orientation)
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS

        int_marker.controls.append(control)

        self._server.insert(int_marker, self._interactive_feedback_cb)

        self._server.applyChanges()

        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = rospy.Time.now()
        # marker.ns = "waypoint"
        # marker.id = len(self._marker_list)
        # marker.lifetime = rospy.Duration()
        # marker.action = Marker.ADD
        # marker.pose = pos

        # marker.scale.x = 5
        # marker.scale.y = 1
        # marker.scale.z = 1

        # marker.color.r = 1.0
        # marker.color.g = 0.0
        # marker.color.b = 0.0
        # marker.color.a = 1.0

        # marker.type = Marker.ARROW

        # return marker

    def run(self, rate: int = 10):
        sleep_rate = rospy.Rate(rate)

        # while not rospy.is_shutdown():
        #     # self._waypoints_pub.publish(self._marker_list)
        #     sleep_rate.sleep()

        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("horiokart_waypoint_marker")

    path = rospy.get_param("~save_path")
    file = rospy.get_param("~exist_file_path", None)

    waypoint_loader = None
    if file is not None and file != "":
        waypoint_loader = WaypointLoaderCSV(path=file)

    maker = WaypointMaker(path=path, waypoint_loader=waypoint_loader)
    maker.run()
