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

## 現在フェーズ: Phase 3（ループクロージャ導入）

Phase 3 の実装対象:

- `optimizer/gtsam_optimizer.py`
- `pose_graph/loop_closure_builder.py`

Phase 1・2 完了済み（Odom SLAM・スキャンマッチング動作確認済み）。

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

### 5. アダプター層がセンサーフレームを正規化する

`input/ros2/ros_adapter.py` が `ScanData` を生成する時点で、スキャン角度を
base_link フレームに正規化すること。センサーの取付け回転は `/tf_static`（URDF から
`robot_state_publisher` が配信）から取得し、YAML パラメータ化しない。
センサー物理配置の単一の正解は URDF にあり、YAML に二重管理すると URDF との乖離が
実行時まで検出できない。
コアロジック（`pose_graph/`, `map_manager/` 等）は `ScanData.angle_min` が
base_link フレームであることを前提としてよい。

### 6. ROS パラメータは 4 箇所を同期させる

`slam_node.py` に新しいパラメータを追加するとき、以下を**全て**更新すること:

1. `config.py` — `SlamConfig` フィールド（デフォルト値を設定する）
2. `slam_node.py` — `_declare_params()` に `self.declare_parameter(...)` を追記
3. `slam_node.py` — `_build_config()` に `self.get_parameter(...).value` を追記
4. `params/slam_gnss_2d.yaml` — パラメータエントリを追記

`declare_parameter` より前に `get_parameter` を呼ぶと `ParameterNotDeclaredException` が発生する。
`_declare_params()` と `_build_config()` の更新は同一コミットで行うこと。

### 7. 反復収束コンポーネントには streak fallback を設ける

自身の出力を次回入力の起点とするコンポーネント（ICP 初期値・ループクロージャ候補スコアなど）は
収束失敗が連鎖する **spiral of doom** のリスクがある。
対策として `failure_streak` カウンタと上限到達時のフォールバック処理を設けること:

```python
if not result.converged:
    self._failure_streak += 1
    if self._failure_streak < self._max_failure_streak:
        return None  # まだ様子見: 今フレームをスキップ
    # 上限到達: 低信頼度エッジとして受け入れ、spiral を脱出
    self._failure_streak = 0
    ...  # odom フォールバック処理
else:
    self._failure_streak = 0  # 成功したらリセット
```

`ScanMatchingBuilder` の実装を参照すること。

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
