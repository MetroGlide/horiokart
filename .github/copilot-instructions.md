## プロジェクト概要

チーム MetroGlide による自律移動台車ロボット MG-01 の開発プロジェクト。詳細は [README.md](../README.md) を参照。

## エージェント向けルール

- ファイルの削除・git操作は行わない
- コメントはコードの意図のみ記載。差分記録・履歴コメントは禁止
- 指示と無関係な箇所は変更しない
- 後方互換性は指示がない限り考慮しない
- 指示のないコード整形・リンターエラー修正は行わない

## 実行環境

**ローカルにROS2環境はない。全操作はDockerコンテナ内で行う。**

```bash
make develop          # developコンテナ起動（バックグラウンド）
make shell-develop    # developコンテナにbashアクセス
make shell svc=slam   # 実行中コンテナにアクセス
make build svc=slam   # イメージビルド（collect_deps.sh自動実行）
make test             # 全テスト実行
make test pkg=<pkg>   # 特定パッケージのテスト
```

コンテナ内でROS2コマンドを使う場合:

```bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
```

ホスト→コンテナのマウント: `/home/chuson/ros_workspace/mg_robot/` → `/app/`

## コーディングスタイル

- Python: PEP8 / C++: Google C++ Style Guide
- パッケージ: モノレポ構成（`/app/` 以下が各ROSパッケージ）

## パッケージ構成

| パッケージ               | 役割                                                           |
| ------------------------ | -------------------------------------------------------------- |
| `mg_bringup`             | slam/navigationを束ねるトップレベルlaunch群                    |
| `mg_description`         | URDF・RViz設定                                                 |
| `mg_diagnostics`         | `/diagnostics`トピックへの正常性診断配信                       |
| `mg_drivers`             | LiDAR/DepthCam/GPS/IMU/モータドライバ群                        |
| `mg_msgs`                | カスタムメッセージ・サービス定義                               |
| `mg_navigation`          | Nav2ラッパー(collision_behavior, AMCL watchdog, GNSS初期化)    |
| `mg_scenario_test`       | GazeboシミュレーションでシナリオYAMLを実行する結合テストFW     |
| `mg_simulation`          | Gazebo Fortress ワールド・launch設定                           |
| `mg_simulator_client`    | シミュレータ操作クライアント                                   |
| `mg_slam`                | slam_toolbox + KISS-ICP launch・パラメータ                     |
| `mg_ui`                  | Web UI / TUI / system_manager の3サブパッケージ                |
| `mg_utils`               | `LaunchArgumentCreator` ヘルパー                               |
| `mg_waypoint_navigation` | FSMベースのウェイポイントシーケンサ(Nav2 ActionClientラッパー) |
| `nav2_pkg`               | **カスタム修正済み**のNav2（アップストリームと差分あり）       |

## 重要なコード規約

### launch ファイル

launch引数は必ず `LaunchArgumentCreator` 経由で定義する（[mg_utils/launch_argument.py](../mg_utils/mg_utils/launch_argument.py)）:

```python
from mg_utils.launch_argument import LaunchArgumentCreator

def generate_launch_description():
    arg = LaunchArgumentCreator()
    simulation = arg.create("simulation", default="true")
    return LaunchDescription([
        *arg.get_created_declare_launch_args(),
        ...
    ])
```

`SIMULATION` 環境変数で実機/シミュを切り替え: `EnvironmentVariable("SIMULATION")`

### パラメータYAML

```yaml
node_name:
  ros__parameters:
    param_key: value
```

## カスタムメッセージ・サービス

定義ファイル: [mg_msgs/msg/](../mg_msgs/msg/)、[mg_msgs/srv/](../mg_msgs/srv/)

## mg_ui

[mg_ui/README.md](../mg_ui/README.md) を参照。

| サブパッケージ      | 技術                                             |
| ------------------- | ------------------------------------------------ |
| `mg_web_ui`         | React 18 + TypeScript + Vite + Tailwind CSS      |
| `mg_tui`            | Python TUI ([README](../mg_ui/mg_tui/README.md)) |
| `mg_system_manager` | FastAPI + Docker SDK（ROS2非依存）               |

**mg_web_ui フロントエンド:**

- ROS通信: foxglove_bridge `ws://localhost:8765`
- 主要フック: `useFoxgloveClient`, `useTopicSubscriber`, `useServiceCaller`, `useNav2Status`, `useSystemManagerClient`
- トピック・サービス定義: [ros/topics.ts](../mg_ui/mg_web_ui/frontend/src/ros/topics.ts), [ros/services.ts](../mg_ui/mg_web_ui/frontend/src/ros/services.ts)
- foxglove経由でROS型を扱う場合はschema名が必要 → [ros/schemas.ts](../mg_ui/mg_web_ui/frontend/src/ros/schemas.ts) を参照

## テスト

```bash
make test             # Dockerコンテナ内で全パッケージテスト
make test pkg=mg_waypoint_navigation
```
