import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node

class SlamTfBroadcaster:
    """map -> odom 変換を計算しブロードキャストする"""

    def __init__(self, node: Node):
        self._node = node
        self._tf_broadcaster = TransformBroadcaster(node)
        self._map_to_odom_x = 0.0
        self._map_to_odom_y = 0.0
        self._map_to_odom_yaw = 0.0

    def update(self, node_pose, odom) -> None:
        """最新キーフレームから map->odom 変換を更新する"""
        c_o = math.cos(odom.yaw)
        s_o = math.sin(odom.yaw)
        inv_x = -(c_o * odom.x + s_o * odom.y)
        inv_y = -(-s_o * odom.x + c_o * odom.y)
        c_m = math.cos(node_pose.yaw)
        s_m = math.sin(node_pose.yaw)
        self._map_to_odom_x = node_pose.x + c_m * inv_x - s_m * inv_y
        self._map_to_odom_y = node_pose.y + s_m * inv_x + c_m * inv_y
        self._map_to_odom_yaw = node_pose.yaw - odom.yaw

    def publish(self) -> None:
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self._map_to_odom_x
        t.transform.translation.y = self._map_to_odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation.w = math.cos(self._map_to_odom_yaw / 2.0)
        t.transform.rotation.z = math.sin(self._map_to_odom_yaw / 2.0)
        self._tf_broadcaster.sendTransform(t)
