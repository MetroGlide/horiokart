from __future__ import annotations

from typing import Dict, List

import yaml

from mg_scenario_test.scenario import (
    EventSpec,
    GoalEventSpec,
    GoalSpec,
    ModelSpec,
    ObstacleDef,
    PoseSpec,
    Scenario,
)


class ScenarioLoader:
    @staticmethod
    def load(path: str) -> Scenario:
        with open(path, "r") as f:
            raw = yaml.safe_load(f)
        return ScenarioLoader._parse(raw)

    @staticmethod
    def _parse(raw: dict) -> Scenario:
        has_goals = "goals" in raw
        has_waypoints_file = "waypoints_file" in raw

        if has_goals and has_waypoints_file:
            raise ValueError(
                "'goals' and 'waypoints_file' are mutually exclusive")
        if not has_goals and not has_waypoints_file:
            raise ValueError(
                "Either 'goals' or 'waypoints_file' must be specified")
        if has_goals and "goal_events" in raw:
            raise ValueError(
                "'goal_events' is only valid with 'waypoints_file'")

        obstacles: Dict[str, ObstacleDef] = {
            name: ObstacleDef(model=ScenarioLoader._parse_model(od["model"]))
            for name, od in raw.get("obstacles", {}).items()
        }

        goals = None
        if has_goals:
            goals = [ScenarioLoader._parse_goal(g) for g in raw["goals"]]

        goal_events = None
        if "goal_events" in raw:
            goal_events = [
                ScenarioLoader._parse_goal_event(ge) for ge in raw["goal_events"]
            ]

        return Scenario(
            version=str(raw.get("version", "1.0")),
            scenario_name=raw.get("scenario_name", ""),
            world_name=raw.get("world_name", "warehouse"),
            obstacles=obstacles,
            goals=goals,
            waypoints_file=raw.get("waypoints_file"),
            goal_events=goal_events,
        )

    @staticmethod
    def _parse_model(raw: dict) -> ModelSpec:
        return ModelSpec(
            type=raw["type"],
            uri=raw.get("uri", ""),
            path=raw.get("path", ""),
            shape=raw.get("shape", ""),
            size=raw.get("size", {}),
        )

    @staticmethod
    def _parse_pose(raw: dict) -> PoseSpec:
        return PoseSpec(
            frame=raw.get("frame", "absolute"),
            x=float(raw.get("x", 0.0)),
            y=float(raw.get("y", 0.0)),
            z=float(raw.get("z", 0.0)),
            yaw=float(raw.get("yaw", 0.0)),
        )

    @staticmethod
    def _parse_event(raw: dict) -> EventSpec:
        pose = ScenarioLoader._parse_pose(
            raw["pose"]) if "pose" in raw else None
        spawn_pose = (
            ScenarioLoader._parse_pose(
                raw["spawn_pose"]) if "spawn_pose" in raw else None
        )
        return EventSpec(
            type=raw["type"],
            pose=pose,
            sec=float(raw.get("sec", 0.0)),
            obstacle=raw.get("obstacle", ""),
            spawn_pose=spawn_pose,
        )

    @staticmethod
    def _parse_goal(raw: dict) -> GoalSpec:
        return GoalSpec(
            pose=ScenarioLoader._parse_pose(raw["pose"]),
            before=[ScenarioLoader._parse_event(
                e) for e in raw.get("before", [])],
            during=[ScenarioLoader._parse_event(
                e) for e in raw.get("during", [])],
            after=[ScenarioLoader._parse_event(e)
                   for e in raw.get("after", [])],
        )

    @staticmethod
    def _parse_goal_event(raw: dict) -> GoalEventSpec:
        return GoalEventSpec(
            waypoint_index=int(raw["waypoint_index"]),
            before=[ScenarioLoader._parse_event(
                e) for e in raw.get("before", [])],
            during=[ScenarioLoader._parse_event(
                e) for e in raw.get("during", [])],
            after=[ScenarioLoader._parse_event(e)
                   for e in raw.get("after", [])],
        )
