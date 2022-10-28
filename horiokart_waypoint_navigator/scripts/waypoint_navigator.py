#!/usr/bin/env python3

#from dataclasses import dataclass
from typing import List
from abc import ABCMeta, abstractmethod
import csv
import math

import rospy
import actionlib
#import tf2_ros
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse


# @dataclass
# class Waypoint():
#    index: int
#    pose: PoseStamped
#    flag: int
class Waypoint():
    def __init__(self, index: int, pose: PoseStamped, flag: int) -> None:
        self.index = index
        self.pose = pose
        self.flag = flag


# todo?:ジェネレータにする？
class WaypointLoaderBase(metaclass=ABCMeta):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def _load(self):
        raise NotImplementedError()

    @abstractmethod
    def get_next_waypoint(self) -> Waypoint:
        raise NotImplementedError()

    @abstractmethod
    def back_point(self, num: int = 1) -> None:
        raise NotImplementedError()


class WaypointLoaderCSV(WaypointLoaderBase):
    def __init__(self, path: str) -> None:
        super().__init__()

        self._path = path

        rospy.loginfo(f"Waypoints Load from {self._path}")
        self._waypoints: List[Waypoint] = self._load()
        self._current_waypoint: int = -1

        rospy.loginfo(f"Load:{len(self._waypoints)} points")

    def _load(self) -> List[Waypoint]:
        waypoint_list = []

        with open(self._path, 'r') as f:
            reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
            for i, p in enumerate(reader):
                rospy.loginfo(f"row:{p}")
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"

                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = p[2]

                pose.pose.orientation.x = p[3]
                pose.pose.orientation.y = p[4]
                pose.pose.orientation.z = p[5]
                pose.pose.orientation.w = p[6]

                waypoint_list.append(Waypoint(
                    index=i,
                    pose=pose,
                    flag=0
                ))

        return waypoint_list

    @property
    def current_waypoint_index(self):
        return self._current_waypoint

    def get_next_waypoint(self) -> Waypoint:
        self._current_waypoint += 1
        if self._current_waypoint >= len(self._waypoints):
            return None
        return self._waypoints[self._current_waypoint]

    def get_all_waypoints(self) -> List[Waypoint]:
        return self._waypoints

    def seek(self):
        ...

    def back_point(self, num: int = 1) -> None:
        self._current_waypoint = max(-1, self._current_waypoint-num)


class WaypointNavigator():
    def __init__(self, waypoint_loader: WaypointLoaderBase) -> None:
        self._waypoint_loader = waypoint_loader

        self._client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        self._client.wait_for_server()

        self._reached = False
        self._last_reached_point = -1

        #self._tf2_buffer = tf2_ros.Buffer()
        #self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)
        self._tf_listener = tf.TransformListener()

        self._is_stop = True
        self._stop_srv = rospy.Service(
            "~stop",
            SetBool,
            self._stop_srv_cb
        )

    def _stop_srv_cb(self, req: SetBoolRequest):
        self._is_stop = req.data
        res = SetBoolResponse()
        res.success = True
        res.message = ""
        return res

    def _make_goal(self, waypoint: Waypoint) -> MoveBaseGoal:
        goal = MoveBaseGoal()
        goal.target_pose = waypoint.pose
        return goal

    def _calc_distance(self, pos1: Point, pos2: Point):
        return math.sqrt((pos1.x-pos2.x)**2 + (pos1.y-pos2.y)**2)

    def _check_reach_point(self, waypoint: Waypoint):
        # try:
        #    trans: TransformStamped = self._tf2_buffer.lookup_transform(
        #        "base_footprint", "map", rospy.Time(0))
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    return False
        try:
            (trans, rot) = self._tf_listener.lookupTransform(
                "map", "base_footprint", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("can't get transform")
            rospy.logerr(e)

        pos = Point()
        pos.x = trans[0]
        pos.y = trans[1]
        pos.z = trans[2]

        dist = self._calc_distance(
            # waypoint.pose.pose.position, trans.transform.translation)
            waypoint.pose.pose.position, pos)
        rospy.logdebug(f"distance {dist}[m]")

        if dist <= 2.0:  # todo
            self._reached = True

    def _aborted_action(self):

        rospy.loginfo(f"Aborted!")
        n = self._waypoint_loader.current_waypoint_index - self._last_reached_point

        # first abort
        if n == 1:
            # try next point by get_next_waypoint() in continued loop
            rospy.loginfo(f"try recovery... plan No.1 skip point")
            pass

        # abort when tryed skip waypoint once
        elif n == 2:
            # try previous reached point
            rospy.loginfo(f"try recovery... plan No.2 back point")
            self._waypoint_loader.back_point(num=3)

        else:
            rospy.logerr(
                f"Can't reach point {self._waypoint_loader.current_waypoint_index}")
            exit(0)

    def run(self, rate: int = 5):
        rospy.sleep(5)  # todo!

        sleep_rate = rospy.Rate(rate)

        # ----------
        # main loop
        # ----------
        while not rospy.is_shutdown():

            # pre process for send goal
            if self._is_stop:
                rospy.loginfo(f"Waiting robot start")
                sleep_rate.sleep()
                continue

            # get next goal and send
            waypoint = self._waypoint_loader.get_next_waypoint()
            if waypoint is None:  # todo
                rospy.loginfo("Goal!")
                exit(0)
                break

            rospy.loginfo(f"Send goal No.{waypoint.index}")
            self._client.send_goal(self._make_goal(waypoint))
            self._reached = False

            # ----------
            # moving loop
            # ----------
            while not self._reached:
                state = self._client.get_state()

                # action abort
                if state == GoalStatus.ABORTED:
                    self._aborted_action()
                    break

                # todo: recovery
                # todo: calc abort from tf

                # check emergency stop
                if self._is_stop:
                    self._client.cancel_goal()
                    self._waypoint_loader.back_point()
                    break

                sleep_rate.sleep()

                self._check_reach_point(waypoint=waypoint)

            # post process for point reached
            if self._reached:
                self._last_reached_point = waypoint.index
                rospy.loginfo(f"Reach point No.{waypoint.index}")
                rospy.loginfo(f"Set next goal!")


if __name__ == '__main__':
    rospy.init_node('horiokart_waypoint_navigation')

    path = rospy.get_param("~waypoint_path")

    waypoint_loader = WaypointLoaderCSV(path=path)
    waypoint_navigator = WaypointNavigator(waypoint_loader=waypoint_loader)
    waypoint_navigator.run()
