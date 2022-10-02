#!/usr/bin/env python3

import psutil

import rospy
from std_msgs.msg import Float32


class PcResourcePublisher(object):
    def __init__(self) -> None:
        self._cpu_pub = rospy.Publisher(
            "~cpu",
            Float32,
            queue_size=1
        )
        self._mem_pub = rospy.Publisher(
            "~memory",
            Float32,
            queue_size=1
        )

        self._rate: int = rospy.get_param("~rate", 1)
        self._cpu_interval: float = rospy.get_param("~cpu_interval", 0.5)

        if self._cpu_interval > 1/self._rate:
            rospy.logerr("CPU interval[sec] over publish rate!!!")

    def _publish(self):
        mem = psutil.virtual_memory()
        rospy.loginfo(f"memory:{mem.percent}")
        self._mem_pub.publish(
            Float32(
                data=mem.percent
            )
        )

        cpu = psutil.cpu_percent(interval=self._cpu_interval)
        rospy.loginfo(f"cpu:{cpu}")
        self._cpu_pub.publish(
            Float32(
                data=cpu
            )
        )

    def run(self):
        rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()


if __name__=="__main__":
    rospy.init_node("pc_resource_pulisher")
    prp = PcResourcePublisher()
    prp.run()




