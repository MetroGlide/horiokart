from __future__ import annotations

from abc import ABC, abstractmethod

from mg_scenario_test.scenario import ModelSpec, PoseSpec


class SimulatorAdapter(ABC):
    @abstractmethod
    def set_robot_pose(self, name: str, pose: PoseSpec) -> bool:
        ...

    @abstractmethod
    def spawn_entity(self, name: str, model: ModelSpec, pose: PoseSpec) -> bool:
        ...

    @abstractmethod
    def despawn_entity(self, name: str) -> bool:
        ...
