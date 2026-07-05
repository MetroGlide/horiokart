from __future__ import annotations

from typing import TYPE_CHECKING

from mg_waypoint_navigation.waypoint_sequencer.actions.base import BaseAction
from mg_waypoint_navigation.waypoint_sequencer.actions.builtins import (
    AmclResetAction,
    LoadMapAction,
    WaitAction,
    WaitTriggerAction,
    SetNavigationModeAction,
)
from mg_waypoint_navigation.waypoint_sequencer.actions.generic import (
    GenericPublishAction,
    GenericServiceAction,
)

if TYPE_CHECKING:
    import rclpy.node
    from mg_waypoint_navigation.waypoint import ActionConfig

_ACTION_REGISTRY = {
    "service": GenericServiceAction,
    "publish": GenericPublishAction,
    "load_map": LoadMapAction,
    "amcl_reset": AmclResetAction,
    "wait": WaitAction,
    "wait_trigger": WaitTriggerAction,
    "set_navigation_mode": SetNavigationModeAction,
}

def build_action(config: "ActionConfig", node: "rclpy.node.Node") -> BaseAction:
    cls = _ACTION_REGISTRY.get(config.type)
    if cls is None:
        raise ValueError(f"Unknown action type: {config.type!r}")
    return cls(config, node)

__all__ = [
    "BaseAction",
    "build_action",
    "GenericServiceAction",
    "GenericPublishAction",
    "LoadMapAction",
    "AmclResetAction",
    "WaitAction",
    "WaitTriggerAction",
    "SetNavigationModeAction",
]
