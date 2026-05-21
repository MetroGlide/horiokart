---
description: "Use when working on slam_gnss_2d: a GNSS-constrained 2D SLAM implementation. Covers architecture, ABC interfaces, swappable components, and development phases."
applyTo: "mg_slam/scripts/slam_gnss_2d/**"
---

# slam_gnss_2d 開発ガイド

**作業前に必ず以下の設計ドキュメントを読むこと:**

- [全体アーキテクチャ設計](../../doc/slam_gnss_2d_design.md)
  — コンポーネント構成・ABC仕様・データフロー・差し替えポイント
- [開発フェーズ定義](../../doc/slam_gnss_2d_phases.md)
  — 現在フェーズ・実装スコープ・完了条件

---

## 現在フェーズ: Phase 1（オドメトリのみでマップ作成）

Phase 1 の実装対象:

- `data_types.py` / `input/base.py` / `input/ros2/ros_adapter.py`
- `pose_graph/odom_builder.py` / `map_manager/opencv_renderer.py` / `slam_node.py`

Phase 2〜4 のファイルは ABC スタブのみ（`raise NotImplementedError`）。

---

## 絶対に守るべき設計ルール

### 1. コアロジックは ROS に完全非依存

`pose_graph/` `scan_matching/` `gnss/` `optimizer/` `map_manager/` では `import rclpy` を使わない。
ROSメッセージ型（`LaserScan`, `Odometry` 等）をこれらのファイルに持ち込まない。

ROSに触れてよいのは以下のファイルのみ:

- `input/ros2/ros_adapter.py`
- `input/ros2/bag_reader.py`
- `slam_node.py`

### 2. 全コンポーネントは ABC を継承する

新しい実装を追加する場合、必ず対応する `base.py` の ABC を継承すること。
ABC のメソッドシグネチャを変更する場合は先に `slam_gnss_2d_design.md` を更新すること。

### 3. 未実装フェーズのコードには `raise NotImplementedError` を置く

`pass` は使わない。フェーズを明示したメッセージを付けること:

```python
raise NotImplementedError("ICPMatcher は Phase 2 で実装予定です")
```

### 4. GNSS 拘束は Phase 4 でのみ追加

Phase 1〜3 では GNSS データをポーズグラフの拘束に使わない。
Phase 1〜3 では `GnssSourceBase` を使わない。

---

## データフロー

```
[Input Layer]  LaserScan/Odometry/NavSatFix
      ↓ (dataclass変換)
[data_types]   ScanData / OdomData / GnssData
      ↓
[pose_graph]   PoseNode (x, y, yaw, scan)
      ↓
[map_manager]  numpy array → OccupancyGrid
      ↓
[slam_node]    ROS2 Publisher
```

---

## 差し替えポイント一覧

| 変更したいもの                        | 差し替えるクラス                          | ABC                    |
| ------------------------------------- | ----------------------------------------- | ---------------------- |
| オドメトリソース (/odom → /odom/gnss) | `ROS2OdomSource(topic=...)`               | `OdomSourceBase`       |
| ポーズグラフ構築アルゴリズム          | `OdomOnlyBuilder` → `ScanMatchingBuilder` | `PoseGraphBuilderBase` |
| スキャンマッチング実装                | `ICPMatcher` → 他                         | `ScanMatcherBase`      |
| GNSS整合アルゴリズム                  | `KinematicHeadingAligner` → 他            | `GnssAlignerBase`      |
| グラフ最適化ライブラリ                | `GTSAMOptimizer` → 他                     | `GraphOptimizerBase`   |
| マップ描画実装                        | `OpenCVRenderer` → 他                     | `MapRendererBase`      |
| ミドルウェア全体                      | `input/ros2/` → `input/other/`            | 各 Source ABC          |

差し替えは `slam_node.py` の依存生成部分のみを変更することで完結するよう設計されている。

---

## パッケージインポート規約

`slam_node.py` から始まるすべての絶対インポートは `slam_gnss_2d.` プレフィックスを使う:

```python
from slam_gnss_2d.data_types import ScanData
from slam_gnss_2d.input.ros2.ros_adapter import ROS2ScanSource
```

`slam_gnss_2d/` 内部のファイルは相対インポートを使う:

```python
from ..data_types import ScanData
from .base import PoseGraphBuilderBase
```
