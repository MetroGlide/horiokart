#!/usr/bin/env python3

import enum

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_msgs.msg import Bool


class BoolSubscriber:
    def __init__(self, topic_name: str, on_change_callback=None) -> None:
        self._topic_name = topic_name
        self._on_change_cb = on_change_callback

        self._pre_msg_state = None
        self._sub = rospy.Subscriber(
            self._topic_name,
            Bool,
            self._msg_cb
        )

    def _msg_cb(self, msg: Bool):
        if self._pre_msg_state is not None and self._pre_msg_state == msg.data:
            return

        self._pre_msg_state = msg.data
        if self._on_change_cb is not None:
            self._on_change_cb(self._topic_name)

    def get_data(self):
        return self._pre_msg_state


class NavigationInterface():
    """
        - robot start/stop
    """
    class State(enum.IntEnum):
        MOVE = enum.auto()
        STOP = enum.auto()

    def __init__(self) -> None:

        self._robot_stop_srv = rospy.ServiceProxy(
            "/horiokart_waypoint_navigator/stop",
            SetBool
        )
        self._robot_stop_srv.wait_for_service()

        self._stop_buttons = [
            BoolSubscriber("/horiokart2021_sensors/emergency1",
                           self._on_stop_button_change_cb)
        ]

    def _on_stop_button_change_cb(self, topic_name: str):
        buttons_state = [x.get_data() for x in self._stop_buttons]
        if all(buttons_state):
            rospy.loginfo("Stop button pushed")
            self._robot_stop_srv(
                SetBoolRequest(data=True)
            )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('horiokart_navigation_interface')

    manager = NavigationInterface()
    manager.run()
