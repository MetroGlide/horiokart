# SLAM-GNSS-2D ビジュアライゼーション解説

SLAM-GNSS-2D ページで表示される各要素の意味と色の対応をまとめる。

---

## 描画要素の詳細

### 白い点（Pose Graph ノード）

- **色**: 白 `rgb(1.0, 1.0, 1.0)`。最新ノードのみ水色 `rgb(0.0, 1.0, 1.0)`
- **意味**: スキャンを採択したキーフレームの推定位置。スキャン間距離・回転量が閾値 (`min_translation`, `min_rotation`) を超えた時点で追加される
- **実装**: `slam_node_base.py` → `_publish_pose_graph_markers()` の `ns='nodes'` マーカー

### 水色の線（シーケンシャルエッジ）

- **色**: 青みがかった水色 `rgb(0.2, 0.5, 1.0)`
- **意味**: 時系列順に隣接するノード間のスキャンマッチング拘束。白い点（ノード）同士を結ぶ線がこれにあたる。連続フレーム間の相対変換をスキャンマッチングで推定した結果を表す
- **実装**: `slam_node_base.py` → `_publish_pose_graph_markers()` の `ns='seq_edges'` マーカー

### 緑の線（ループエッジ）

- **色**: 緑 `rgb(0.0, 1.0, 0.4)`。シーケンシャルエッジより太い
- **意味**: ループクロージャ検出で追加された非連続ノード間の拘束。離れた場所でスキャンが一致した場合に追加され、グラフ最適化のトリガーとなる
- **実装**: `loop_closure_builder.py` の `_loop_edges` → `ns='loop_edges'` マーカー

### 水色の軌跡線（Path）

- **色**: シアン `#00ffff`
- **意味**: 全ノードの推定位置を時系列に繋いだロボット軌跡。ループ閉合・GNSS最適化後はグラフ全体を再構築した後の**最終軌跡**を表す
- **トピック**: `slam_gnss_2d/path`
- **実装**: `slam_node_base.py` → `_publish_path()` / `_rebuild_path()` → `PathLine.tsx`

### グレーの軌跡線（Pre-optimize Path）

- **色**: 暗いグレー `#666666`
- **意味**: GNSS最適化が走る直前の軌跡スナップショット。最適化前後の軌跡変化を比較するために表示する。ループ閉合時と GNSS 最適化直前の2タイミングで更新される
- **トピック**: `slam_gnss_2d/path_before_optimize`
- **実装**: `slam_node_base.py` → `_path_before_pub.publish(self._path_msg)`

### マゼンタの点（GNSS Points）

- **色**: マゼンタ `rgb(1.0, 0.0, 1.0)`
- **意味**: bag ファイルから読み込んだ NavSatFix を UTM 平面直角座標に変換し、さらに KinematicHeadingAligner の推定変換で SLAM 座標系に投影した GNSS 測位点群。SLAM 軌跡との整合性を目視確認するために表示する
- **トピック**: `slam_gnss_2d/gnss_raw_markers`
- **実装**: `slam_offline_node.py` → `_publish_gnss_raw_markers()`

### 紫ピンクの短い線分（GNSS Constraints）

- **色**: 紫ピンク `rgb(0.8, 0.0, 0.8)`
- **意味**: GNSS 拘束の残差ベクトル。**線分の一方の端が GTSAM 最適化後のノード位置（最終 Pose）**、もう一方の端が GNSS 測位に基づく拘束目標位置。線分が短いほど GNSS 拘束が満足されている
- **トピック**: `slam_gnss_2d/gnss_prior_markers`
- **実装**: `slam_offline_node.py` → `_publish_gnss_prior_markers()`

> **補足**: ポーズグラフのノード側が最終 Pose であり、GNSS 測位点側は拘束のターゲット座標であって最終位置ではない。

---

## データフロー（オフライン実行時）

```
bag
 ├─ BagScanSource  ──→ add_scan() ──→ PoseGraph ──→ _publish_pose_graph_markers()
 │                                                      白点  : ノード位置
 │                                                      水色線: seq_edges（連続フレーム拘束）
 │                                                      緑線  : loop_edges（ループクロージャ拘束）
 │
 └─ BagGnssSource.start()
         │ UTM 変換
         │ KinematicHeadingAligner.estimate_transform()  → SLAM 座標系へ投影
         ↓
    _publish_gnss_raw_markers()                          → マゼンタ点
         ↓
    GnssConstraintInserter.build_priors()
    GTSAMOptimizer.optimize()
         ↓  updated_nodes（最終 Pose）
    _publish_gnss_prior_markers()                        → 紫ピンク線
    _rebuild_path(updated_nodes)                         → 水色軌跡線（最終）
```

---

## トピック一覧

| トピック                            | 型                               | 配信元                          |
| ----------------------------------- | -------------------------------- | ------------------------------- |
| `slam_gnss_2d/map`                  | `nav_msgs/OccupancyGrid`         | `SlamNodeBase`                  |
| `slam_gnss_2d/path`                 | `nav_msgs/Path`                  | `SlamNodeBase`                  |
| `slam_gnss_2d/path_before_optimize` | `nav_msgs/Path`                  | `SlamNodeBase`                  |
| `slam_gnss_2d/pose_graph`           | `visualization_msgs/MarkerArray` | `SlamNodeBase`                  |
| `slam_gnss_2d/gnss_raw_markers`     | `visualization_msgs/MarkerArray` | `SlamOfflineNode`               |
| `slam_gnss_2d/gnss_prior_markers`   | `visualization_msgs/MarkerArray` | `SlamOfflineNode`               |
| `/gps/fix`                          | `sensor_msgs/NavSatFix`          | `SlamOfflineNode`（bag 再配信） |
