"""ROS2 依存モジュールを sys.modules でモックし、ROS2 なしで pytest を実行可能にする。"""
import sys
from unittest.mock import MagicMock

_ROS2_MODULES = [
    "rclpy",
    "rclpy.node",
    "rclpy.action",
    "rclpy.action.client",
    "nav2_msgs",
    "nav2_msgs.action",
    "nav2_msgs.srv",
    "action_msgs",
    "action_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "ament_index_python",
    "ament_index_python.packages",
    "tf2_ros",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
    "nav_msgs",
    "nav_msgs.msg",
    "visualization_msgs",
    "visualization_msgs.msg",
    "mg_msgs",
    "mg_msgs.msg",
    "mg_msgs.srv",
    "builtin_interfaces",
    "builtin_interfaces.msg",
]

for _mod in _ROS2_MODULES:
    if _mod not in sys.modules:
        sys.modules[_mod] = MagicMock()
