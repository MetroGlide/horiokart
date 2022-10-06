#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PoseStamped
from horiokart_waypoint_msgs.srv import SaveWaypoint, SaveWaypointRequest, SaveWaypointResponse

import os
import csv
from typing import List


class WaypointMaker():
    def __init__(self, path: str) -> None:
        self._path = path
        self._marker_list: List[Marker] = []

        self._waypoints_pub = rospy.Publisher(
            "waypoint", MarkerArray, queue_size=1)
        self._posestamp_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self._pose_sub_cb)

        self._save_srv = rospy.Service(
            "~save",
            SaveWaypoint,
            self._save_srv_cb
        )

    def _pose_sub_cb(self, data: PoseStamped):
        pos = data.pose
        self._marker_list.append(self._make_marker(pos))

        print(
            f"{pos.position.x},{pos.position.y},0.0,0.0,0.0,{pos.orientation.z},{pos.orientation.w},")

    def _save_srv_cb(self, req: SaveWaypointRequest) -> SaveWaypointResponse:
        path = os.path.join(self._path, req.file_name)
        with open(path, 'w') as f:
            writer = csv.writer(f)
            for p in self._marker_list:
                pos = p.pose.position
                rot = p.pose.orientation
                writer.writerow(
                    [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w]
                )
        res = SaveWaypointResponse()
        res.success = True
        res.path = path
        return res

    def _make_marker(self, pos: Pose):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "waypoint"
        marker.id = len(self._marker_list)
        marker.lifetime = rospy.Duration()
        marker.action = Marker.ADD
        marker.pose = pos

        marker.scale.x = 5
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.type = Marker.ARROW

        return marker

    def run(self, rate: int = 10):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self._waypoints_pub.publish(self._marker_list)
            rate.sleep()

        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("horiokart_waypoint_marker")

    path = rospy.get_param("~save_path")

    maker = WaypointMaker(path=path)
    maker.run()
