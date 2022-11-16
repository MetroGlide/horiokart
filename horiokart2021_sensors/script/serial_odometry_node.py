#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import datetime
import csv
import os
import time

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
        self._csv_path = rospy.get_param("~csv_path", "/root/ros_data/log_csv")
        self._csv_name = rospy.get_param("~csv_name", "")

        if self._is_write_csv:
            if self._csv_name == "":
                dt_now = datetime.datetime.now()
                self._csv_name = dt_now.strftime("odom_%Y%m%d%a_%H%M%S.csv")

            with open(os.path.join(self._csv_path,self._csv_name), "w") as _:
                rospy.loginfo(f"Odom create CSV {os.path.join(self._csv_path,self._csv_name)}")
            
        else:
            rospy.loginfo(f"Odom No CSV")


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

        rospy.logdebug(f"read:{''.join([f'0x{x:02x} ' for x in self._current_data.raw])}")

        if self._current_data.error != SerialError.NoError:
            rospy.logerr(f"ERROR code {self._current_data.error}")

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

        rospy.logdebug(f"x: {self._recent_valid_data.x:.3f} y: {self._recent_valid_data.y:.3f} th: {self._recent_valid_data.th:.3f}")
        rospy.logdebug(f"vx: {self._recent_valid_data.vx:.3f} vy: {self._recent_valid_data.vy:.3f} vth: {self._recent_valid_data.vth:.3f}")

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
        if self._is_write_csv:
            with open(os.path.join(self._csv_path, self._csv_name)) as f:
                writer = csv.writer(f)

                writer.writerow(
                    [
                        time.time(), # epoch milli sec
                        self._current_data.x,
                        self._current_data.y,
                        self._current_data.th,
                        self._current_data.vx,
                        self._current_data.vy,
                        self._current_data.vth,
                        "",
                        self._current_data.error,
                        "",
                        *[f'0x{x:02x}' for x in self._current_data.raw]
                    ]
                )


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
