#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


rospy.init_node("depth_viewer")
bridge = CvBridge()

depth_image = None


def _depth_cb(data):
    rospy.loginfo_throttle(5, "get image")
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")


sb = rospy.Subscriber("/aligned_depth_to_color/image_raw", Image, _depth_cb)

while not rospy.is_shutdown():
    if depth_image is not None:
        cv2.imshow("image", depth_image)
        cv2.waitKey(1)
    else:
        rospy.logwarn_throttle(5, "no image")
