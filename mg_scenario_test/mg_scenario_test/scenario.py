from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class PoseSpec:
    """位置姿勢の指定。絶対座標またはロボット相対座標を選択できる。"""
    frame: str  # "absolute" | "robot_relative"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


@dataclass
class ModelSpec:
    """シミュレータに配置するモデルの定義。"""
    type: str  # "fuel" | "local" | "primitive"
    uri: str = ""         # fuel: "Owner/models/Name" or full Fuel URL
    path: str = ""        # local: ローカルSDFファイルパス
    shape: str = ""       # primitive: "box" | "cylinder" | "sphere"
    size: Dict[str, float] = field(default_factory=dict)  # primitive用寸法


@dataclass
class ObstacleDef:
    """名前付き障害物定義。scenarioのobstaclesセクションで定義する。"""
    model: ModelSpec


@dataclass
class EventSpec:
    """シナリオイベントの定義。typeによって使用するフィールドが異なる。

    type一覧:
      reset_pose              : ロボットをpose位置にテレポート
      set_amcl_initial_pose   : AMCLの初期位置をposeに設定
      delay                   : sec秒待機
      spawn_obstacle          : obstacle(名前)をposeにスポーン
      despawn_obstacle        : obstacle(名前)をデスポーン
      cleanup_all_obstacles   : シナリオ内でスポーンした全障害物をデスポーン
      trigger_waypoint        : waypoint_sequencerの~/startを呼び出して次のwaypointへ進める
      set_sequencer_index     : waypoint_sequencerの次のwaypoint indexを設定(IDLE時のみ有効)
    """
    type: str
    # reset_pose / set_amcl_initial_pose / spawn_obstacle
    pose: Optional[PoseSpec] = None
    sec: float = 0.0                       # delay
    obstacle: str = ""                     # spawn_obstacle / despawn_obstacle
    spawn_pose: Optional[PoseSpec] = None  # spawn_obstacle (後方互換)
    countdown_ms: int = 0                  # trigger_waypoint
    target_index: int = 0                  # set_sequencer_index


@dataclass
class GoalSpec:
    """インライン定義のゴール。before/during/afterイベントを持つ。"""
    pose: PoseSpec
    before: List[EventSpec] = field(default_factory=list)
    during: List[EventSpec] = field(default_factory=list)
    after: List[EventSpec] = field(default_factory=list)


@dataclass
class GoalEventSpec:
    """waypoints_file使用時のper-waypointイベント定義。"""
    waypoint_index: int
    before: List[EventSpec] = field(default_factory=list)
    during: List[EventSpec] = field(default_factory=list)
    after: List[EventSpec] = field(default_factory=list)


@dataclass
class Scenario:
    version: str
    scenario_name: str
    world_name: str
    obstacles: Dict[str, ObstacleDef] = field(default_factory=dict)
    goals: Optional[List[GoalSpec]] = None
    waypoints_file: Optional[str] = None
    waypoints_nav_mode: str = "direct"  # "direct" | "sequencer"
    sequencer_namespace: str = "waypoint_sequencer_node"
    start_waypoint_index: int = 0
    goal_events: Optional[List[GoalEventSpec]] = None
    finally_events: List[EventSpec] = field(default_factory=list)
