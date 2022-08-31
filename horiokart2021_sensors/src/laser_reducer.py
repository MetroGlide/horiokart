#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

import copy

class LaserReducer():
    def __init__(self) -> None:
        rospy.init_node("laser_reducer")
        rospy.loginfo("init node : laser_reducer")


        self._skip_num = rospy.get_param("~skip_num", 2)
        rospy.loginfo(f"skip num : {self._skip_num}")

        rospy.Subscriber("scan", LaserScan, self._get_scan_cb)
        self.pub = rospy.Publisher("/scan_reduced", LaserScan, queue_size=1)

        self._init = False

        rospy.spin()

    def _get_scan_cb(self, scan: LaserScan):
        msg = copy.deepcopy(scan)

        msg.angle_increment = scan.angle_increment * (1 + self._skip_num)

        msg.ranges = msg.ranges[::(1 + self._skip_num)]
        msg.intensities = msg.intensities[::(1 + self._skip_num)]

        if not self._init:
            self._init = not self._init
            rospy.loginfo(f"original num: {len(scan.ranges)}")
            rospy.loginfo(f"after reduce: {len(msg.ranges)}")
        
        self.pub.publish(msg)



if __name__ == '__main__':
    LaserReducer()