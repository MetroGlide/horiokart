#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerServer, InteractiveMarkerFeedback
from geometry_msgs.msg import Pose, PoseStamped
from horiokart_waypoint_msgs.srv import SaveWaypoint, SaveWaypointRequest, SaveWaypointResponse

from waypoint_navigator import Waypoint

import os
import csv
from typing import List, Dict
import copy


class WaypointManager():
    def __init__(self) -> None:
        self._waypoint_dict: Dict[int, Waypoint] = {}

    def get_next_index(self) -> int:
        return self._get_max_index() + 1

    def update_pose(self, index: int, pose: Pose):
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

    def load(self, path: str):
        ...


class WaypointMaker():
    def __init__(self, path: str, file: str = None) -> None:
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

        if file is not None:
            # todo! load
            ...

    def _pose_sub_cb(self, data: PoseStamped):
        pos = data.pose
        # self._marker_list.append(self._make_marker(pos))
        self._make_marker(pos)

        print(
            f"{pos.position.x},{pos.position.y},0.0,0.0,0.0,{pos.orientation.z},{pos.orientation.w},")

    def _interactive_feedback_cb(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo(f"update index {feedback.marker_name}")

            self._waypoint_manager.update_pose(
                index=int(feedback.marker_name),
                pose=feedback.pose
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
                pos = waypoint.pose.position
                rot = waypoint.pose.orientation
                writer.writerow(
                    [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w,  # 0~6
                     waypoint.reach_threshold,  # 7
                     waypoint.is_stop  # 8
                     ]
                )
        res = SaveWaypointResponse()
        res.success = True
        res.path = path
        return res

    def _make_marker(self, pos: Pose):
        index = self._waypoint_manager.get_next_index()

        pos.position.z += 1
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose = pos
        int_marker.name = str(index)

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

        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "move_x"
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

        int_marker.controls.append(rotate_control)

        control = InteractiveMarkerControl()
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
        self._waypoint_manager.append_waypoint(
            Waypoint(
                index=index,
                pose=pos,
                flag=0,
                reach_threshold=2.0,  # todo
                is_stop=False
            )
        )

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

    maker = WaypointMaker(path=path)
    maker.run()
