# slam_gnss_2d GNSS拘束アルゴリズム詳細

## 概要

GNSS拘束処理は、ロボットの起動地点を基準としたSLAMのローカル座標系と、地球座標系（UTM平面直角座標）を整合させるために実行される。
処理は `GraphOrchestrator` と `GnssAnchorManager` が担当する。

## ステップ 1 — GnssAnchorManager: アンカー設定と座標変換

**ファイル**: [anchor_manager.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/gnss/anchor_manager.py)

1. **アンカー設定**: 最初の有効な GNSS fix を受信した時点で、そこを基準アンカーのUTM座標として記録する。
2. **ローカル平面座標への変換**: 以降のGNSS座標は、このアンカー座標との差分をとることで、ローカル平面座標 $(x_{local}, y_{local})$ に変換される。

## ステップ 2 — GraphOrchestrator: 拘束の逐次追加と最適化

**ファイル**: [graph_orchestrator.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/core/graph_orchestrator.py)

### 1. Prior 拘束の追加
スキャンと同時にGNSS測位データが入力された場合、`GraphOrchestrator` は以下の手順で拘束を追加する。
- 共分散から標準偏差 $\sigma_{xy}$ を計算。
- 変換されたローカル座標 $(x_{local}, y_{local})$ を、該当するノードインデックスに対する絶対位置拘束（PriorFactorPose2）として追加。
- GNSSでは方位が観測できないため、yawの分散は非常に大きな値（`gnss_factor_yaw_variance`）に設定される。

### 2. 逐次最適化
[ISAM2Optimizer](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/optimizer/isam2_optimizer.py) が各ノードの追加やループ辺・GNSS拘束の追加が行われるたびに、ファクターグラフ全体の最適化を実行する。

## 初期方位とバッチ最適化（オフライン）

現在のインクリメンタルパイプラインでは、初期方位（ロボットの起動時の向いている絶対方位）の推定など複雑な初期化処理は行わない。
高精度なマップのアライメントは、収集した `pose_graph.json` を用いて、ROS2ノード（[reoptimize_node.py](file:///home/chuson/ros_workspace/mg_robot/mg_slam/scripts/slam_gnss_2d/nodes/reoptimize_node.py)）にて実施する設計となっている。
