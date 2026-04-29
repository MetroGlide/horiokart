"""v1 YAML フォーマットから v2 Waypoint への変換"""
from __future__ import annotations

from geometry_msgs.msg import Point, PoseStamped, Quaternion

from mg_waypoint_navigation.waypoint import ActionConfig, NavigationConfig, Waypoint

_V1_ACTION_MAP = {
    "front_lidar_off": lambda: ActionConfig(
        type="service",
        service="/front_lidar_publish_controller_node/change_publish_state",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": False},
    ),
    "front_lidar_on": lambda: ActionConfig(
        type="service",
        service="/front_lidar_publish_controller_node/change_publish_state",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": True},
    ),
    "amcl_on": lambda: ActionConfig(
        type="service",
        service="/amcl/enable",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": True},
    ),
    "amcl_off": lambda: ActionConfig(
        type="service",
        service="/amcl/enable",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": False},
    ),
    "gps_on": lambda: ActionConfig(
        type="service",
        service="/gnss_odometry_node/change_publish_state",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": True},
    ),
    "gps_off": lambda: ActionConfig(
        type="service",
        service="/gnss_odometry_node/change_publish_state",
        srv_module="std_srvs.srv",
        srv_class="SetBool",
        request={"data": False},
    ),
    "wait_trigger": lambda: ActionConfig(type="wait", countdown_ms=0),
    "reload_map": lambda: ActionConfig(type="load_map"),
    "wait_all_action_done": lambda: ActionConfig(type="wait", countdown_ms=0),
}


def convert_v1_waypoint(wp_raw: dict) -> Waypoint:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position = Point(**wp_raw["pose"]["position"])
    pose.pose.position.z = 1.0
    pose.pose.orientation = Quaternion(**wp_raw["pose"]["orientation"])

    nav = NavigationConfig(
        reach_tolerance=wp_raw.get("reach_tolerance", 0.5),
        through_tolerance=wp_raw.get("through_tolerance", 3.0),
        is_through_point=wp_raw.get("is_through_point", True),
    )

    actions = []
    for action_str in wp_raw.get("on_reached_action", []):
        factory = _V1_ACTION_MAP.get(action_str)
        if factory is not None:
            action = factory()
            if action.type == "load_map":
                action.localization = wp_raw.get("localization_map_yaml", "")
                action.planning = wp_raw.get("planning_map_yaml", "")
            actions.append(action)

    gnss_label = wp_raw.get("gnss_transform_label", "")
    if gnss_label:
        actions.append(
            ActionConfig(
                type="publish",
                topic="/gnss_odometry_node/select_static_transform",
                msg_module="std_msgs.msg",
                msg_class="String",
                data={"data": gnss_label},
            )
        )

    return Waypoint(
        index=wp_raw["index"],
        pose=pose,
        navigation=nav,
        on_reached_actions=actions,
    )
