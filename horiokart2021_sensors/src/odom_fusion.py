#!/usr/bin/env python3
import rospy
import tf.transformations
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

import numpy as np
import copy


def quat_add(q1: Quaternion, q2: Quaternion) -> Quaternion:
    # return Quaternion(q1[0]+q2[0], q1[1]+q2[1], q1[2]+q2[2], q1[3]+q2[3])
    return Quaternion(q1.x+q2.x, q1.y+q2.y, q1.z+q2.z, q1.w+q2.w)


def quat_sub(q1: Quaternion, q2: Quaternion) -> Quaternion:
    return Quaternion(q1.x-q2.x, q1.y-q2.y, q1.z-q2.z, q1.w-q2.w)


def quat_parse(q: Quaternion) -> tuple:
    return (q.x, q.y, q.z, q.w)


class OdometryManager():
    def __init__(self, topic_name: str) -> None:

        self._sub = rospy.Subscriber(
            topic_name,
            Odometry,
            self._sub_cb,
            queue_size=1
        )

        self._pre_global_odom = Odometry()
        self._pre_send_msg = Odometry()

        self._msg = None
        self._is_updated = False
        self._is_updated_global = False

    def _sub_cb(self, msg):
        self._msg = msg
        self._is_updated = True

    def set_odom(self, odom):
        self._global_odom = odom

    def get(self):
        self._is_updated = False
        return self._msg

    def get_diff(self, global_odom):
        if not self._is_updated:
            return None

        # 呼び出し元で保持しているodomの変化分
        global_diff = copy.deepcopy(global_odom)
        global_diff.pose.pose.position.x -= self._pre_global_odom.pose.pose.position.x
        global_diff.pose.pose.position.y -= self._pre_global_odom.pose.pose.position.y
        # global_diff.pose.pose.orientation -= self._pre_global_odom.pose.pose.orientation
        # global_diff.pose.pose.orientation = quat_sub(
        #     global_diff.pose.pose.orientation, self._pre_global_odom.pose.pose.orientation)
        pre_global_odom_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(self._pre_global_odom.pose.pose.orientation)
        ))
        global_diff_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(global_diff.pose.pose.orientation)
        ))
        global_diff_th -= pre_global_odom_th

        # Subscribeした変化分
        local_diff = copy.deepcopy(self._msg)
        local_diff.pose.pose.position.x -= self._pre_send_msg.pose.pose.position.x
        local_diff.pose.pose.position.y -= self._pre_send_msg.pose.pose.position.y
        # local_diff.pose.pose.orientation -= self._pre_send_msg.pose.pose.orientation
        # local_diff.pose.pose.orientation = quat_sub(
        #     local_diff.pose.pose.orientation, self._pre_send_msg.pose.pose.orientation)
        pre_send_msg_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(self._pre_send_msg.pose.pose.orientation)
        ))
        local_diff_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(local_diff.pose.pose.orientation)
        ))
        local_diff_th -= pre_send_msg_th

        # 呼び出し元からみた差分
        diff = local_diff
        diff.pose.pose.position.x -= global_diff.pose.pose.position.x
        diff.pose.pose.position.y -= global_diff.pose.pose.position.y
        # diff.pose.pose.orientation -= self.global_diff.pose.pose.orientation
        # diff.pose.pose.orientation = quat_sub(
        #     diff.pose.pose.orientation, global_diff.pose.pose.orientation)
        local_diff_th -= global_diff_th
        diff.pose.pose.orientation = Quaternion(
            *(tf.transformations.quaternion_from_euler(*local_diff_th))
        )

        self._pre_send_msg = self._msg
        self._is_updated_global = False
        self._is_updated = False

        return diff

    def update_global_odom(self, odom):
        if self._is_updated_global:
            return

        self._pre_global_odom = copy.deepcopy(odom)
        self._is_updated_global = True

    def is_updated(self):
        return self._is_updated


class OdometryFusion():
    def __init__(self) -> None:

        self._init_ros_params()

        self._wheel_odom = OdometryManager("/wheel/odom")
        self._rf2o_odom = OdometryManager("/rf2o_laser_odometry/odom_rf2o")

        self._odom = Odometry()
        self._odom.header.frame_id = self._odom_frame
        self._odom.child_frame_id = self._base_frame

        self._odom_pub = rospy.Publisher(
            "odom",
            Odometry,
            queue_size=1
        )

    def _init_ros_params(self):
        self._pub_rate = rospy.get_param("~publish_rate", 10)
        self._odom_frame = rospy.get_param("~odom_frame", "odom")
        self._base_frame = rospy.get_param("~base_frame", "base_footprint")

    def _publish(self):
        # 基本wheelで新しいデータが来てない場合のみrf2o
        # それぞれ前回呼び出しからの差分を使用
        w_diff = self._wheel_odom.get_diff(self._odom)
        rf_diff = self._rf2o_odom.get_diff(self._odom)

        if w_diff is not None:
            diff = w_diff
            rospy.loginfo(f"publish wheel odom")
        elif rf_diff is not None:
            diff = rf_diff
            rospy.loginfo(f"publish lidar odom")
        else:
            rospy.loginfo(f"no odom")
            return

        self._odom.pose.pose.position.x += diff.pose.pose.position.x
        self._odom.pose.pose.position.y += diff.pose.pose.position.y
        self._odom.pose.pose.position.z += diff.pose.pose.position.z
        # self._odom.pose.pose.orientation += diff.pose.pose.orientation
        # self._odom.pose.pose.orientation = quat_add(
        #     self._odom.pose.pose.orientation, diff.pose.pose.orientation)
        diff_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(diff.pose.pose.orientation)
        ))
        odom_th = np.array(tf.transformations.euler_from_quaternion(
            quat_parse(self._odom.pose.pose.orientation)
        ))
        odom_th += diff_th
        self._odom.pose.pose.orientation = Quaternion(
            *(tf.transformations.quaternion_from_euler(*odom_th))
        )

        self._odom.twist.twist.linear.x = diff.twist.twist.linear.x
        self._odom.twist.twist.linear.y = diff.twist.twist.linear.y
        self._odom.twist.twist.angular.z = diff.twist.twist.angular.z

        self._odom.header.stamp = rospy.Time.now()

        self._odom_pub.publish(
            self._odom
        )

        self._wheel_odom.update_global_odom(self._odom)
        self._rf2o_odom.update_global_odom(self._odom)

    def run(self):
        rate = rospy.Rate(self._pub_rate)

        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("odometry_fusion_node")
    rospy.loginfo("odometry_fusion_node init")

    of = OdometryFusion()
    of.run()
