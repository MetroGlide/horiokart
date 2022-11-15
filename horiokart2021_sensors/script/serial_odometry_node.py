#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

# from horiokart2021_sensors.serial_odometry import SerialOdometry
from horiokart2021_sensors.src.horiokart2021_sensors.serial_odometry import SerialOdometry, SerialCommand, OdometryData
from horiokart2021_sensors.src.horiokart2021_sensors.serial_communicator import SerialCommand, SerialCommunicator, SerialError


class SerialOdometryNode():
    def __init__(self) -> None:

        self._odom_frame = rospy.get_param("~odom_frame", "odom")
        self._base_frame = rospy.get_param("~base_frame", "base_footprint")
        self._pub_rate = rospy.get_param("~odom_pub_rate", 20)

        self._inv_x = rospy.get_param("~inv_x", False)
        self._inv_y = rospy.get_param("~inv_y", False)
        self._inv_th = rospy.get_param("~inv_th", False)

        self._always_publish = rospy.get_param("~always_publis", True)

        self._is_write_csv = rospy.get_param("~csv", True)
        self._csv_name = rospy.get_param("~csv_name", "")

        # todo: prepare csv file

        device_name = rospy.get_param("~device_name", "ttyHoriokart-odom")
        baud = rospy.get_param("~baud", 56700)
        timeout = rospy.get_param("~timeout", 0.5)

        self._serial = SerialCommunicator(
            device_name=device_name,
            baud=baud,
            timeout=timeout
        )
        self._odom = SerialOdometry(communicator=self._serial)

        self._odom_pub = rospy.Publisher(
            "odom",
            Odometry,
            queue_size=1
        )

        self._current_data: OdometryData = None
        self._recent_valid_data: OdometryData = None

    def update_odom(self):
        self._current_data = self._odom.get_odom()

        # todo: print ros log

        if self._current_data.error != SerialError.NoError:
            # todo: print ros log
            ...
        else:
            if self._inv_x:
                self._current_data.x *= -1
            if self._inv_y:
                self._current_data.y *= -1
            if self._inv_th:
                self._current_data.th *= -1

            self._recent_valid_data = self._current_data

    def publish(self):
        if (
            self._current_data.error != SerialError.NoError
                and not self._always_publish
                or self._recent_valid_data is None
        ):
            return

        # todo: print ros debug log

        odom_msg = Odometry()

        odom_msg.header.stamp = rospy.Time().now()
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame

        odom_msg.pose.pose.position.x = self._recent_valid_data.x
        odom_msg.pose.pose.position.y = self._recent_valid_data.y
        odom_msg.pose.pose.position.z = self._recent_valid_data.z

        odom_quat = quaternion_from_euler(0, 0, self._recent_valid_data.th)
        odom_msg.pose.pose.orientation = odom_quat

        odom_msg.twist.twist.linear.x = math.sqrt(
            self._recent_valid_data.vx**2 + self._recent_valid_data.vy**2
        )
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.angular.z = self._recent_valid_data.vth

        self._odom_pub.publish(odom_msg)

    def write_log_csv(self):
        ...

    def run(self):
        rospy.loginfo("start serial odom node")

        e = self._odom.zero_reset(retry=3)
        if e == SerialError.NoError:
            rospy.loginfo("Odometry zero reset DONE")
        else:
            rospy.logwarn("Odometry zero reset ERROR")

        rate = rospy.Rate(self._pub_rate)
        while not rospy.is_shutdown():
            self.update_odom()

            self.publish()

            # todo: write csv

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("serial_odometry_node")

    node = SerialOdometryNode()
    node.run()
