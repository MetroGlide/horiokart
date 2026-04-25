# horiokart

自律移動台車ロボット開発プロジェクト。ROS2 (Humble) + Docker による実行環境を提供します。

## 前提条件

- Docker Engine 24.0+
- Docker Compose v2.20+
- BuildKit 有効（Docker 23.0+ はデフォルトで有効）
- GPU を使用する場合:
  - **Nvidia**: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) のインストールが必要
  - **AMD**: ROCm 対応の `/dev/kfd`, `/dev/dri` が利用可能であること

## セットアップ

```bash
cp .env.example .env
```

`.env` を環境に合わせて編集してください（詳細は[環境変数](#環境変数)を参照）。

## Docker イメージ構成

```
docker/Dockerfile.base
├── runtime      ← slam / navigation / rosbag-replay
│                    ROS2 基本 + RealSense SDK + IMU tools（実機用・軽量）
├── develop      ← develop / rviz2-slam / rviz2-navigation / rviz
│                    runtime + RViz2 + foxglove-bridge + rqt + vim + Qt/ZMQ
└── simulation   ← gazebo-simulation
                     develop + Gazebo Fortress + kisak-mesa
```

## イメージのビルド

### make を使う場合（推奨）

```bash
make build svc=slam              # slam / navigation / rosbag-replay 用（runtime イメージ）
make build svc=gazebo-simulation # Gazebo シミュレーション用（simulation イメージ）
make build svc=develop           # 開発用（develop イメージ）
make build-all                   # 全イメージ一括ビルド
make build-no-cache svc=slam     # キャッシュ無効でビルド
```

`make build` は `docker/collect_deps.sh` を自動実行し、依存解決ファイル（`package.xml` 等）を事前収集してキャッシュを最適化します。

### Docker コマンドを直接使う場合

`make` を使わない場合は、先に依存ファイルを手動で収集してからビルドしてください。

```bash
# 依存ファイルの収集（package.xml / *.rosinstall / requirements.txt 等を docker/deps/ に集める）
bash docker/collect_deps.sh

# ビルド（--target でステージを指定）
docker compose -f compose.yaml build slam              # runtime イメージ
docker compose -f compose.yaml build gazebo-simulation # simulation イメージ
docker compose -f compose.yaml build develop           # develop イメージ

# キャッシュ無効でビルド
docker compose -f compose.yaml build --no-cache slam

# GPU override を適用してビルド（Nvidia の例）
docker compose -f compose.yaml -f compose.gpu.nvidia.yaml build gazebo-simulation
```

## 各サービスの起動

### 起動モード

| コマンド                  | 動作                                           |
| ------------------------- | ---------------------------------------------- |
| `make <service>`          | フォアグラウンドで起動                         |
| `make <service> DETACH=1` | バックグラウンドで起動                         |
| `make develop`            | バックグラウンドで起動（develop のデフォルト） |
| `make develop ATTACH=1`   | バックグラウンド起動後に bash でアタッチ       |

### 実機

```bash
make slam           # SLAM マッピング
make navigation     # 自律ナビゲーション
make rosbag-replay  # rosbag 再生 + nav2
```

> slam / navigation / rosbag-replay は `USE_RVIZ=false` 固定で起動します。  
> RViz2 で可視化する場合は別途 `make rviz2-slam` / `make rviz2-navigation` を使用してください。

### シミュレーション（Gazebo）

```bash
make gazebo-simulation
```

GPU を使用する場合は `.env` の `USE_GPU` を設定するか、コマンドライン引数で上書きできます。

```bash
make gazebo-simulation USE_GPU=nvidia
make gazebo-simulation USE_GPU=amd
```

### RViz2

| コマンド                | 説明                                                      |
| ----------------------- | --------------------------------------------------------- |
| `make rviz2-slam`       | SLAM 用 RViz2 設定で起動                                  |
| `make rviz2-navigation` | ナビゲーション用 RViz2 設定で起動                         |
| `make rviz`             | `.env` の `RVIZ_CONFIG` に指定したコンフィグで RViz2 起動 |

`make rviz` で任意のコンフィグを使用するには `.env` に `RVIZ_CONFIG` を設定します。

```
RVIZ_CONFIG=/root/ros2_ws/src/horiokart/horiokart_navigation/rviz/rviz.rviz
```

### 開発

```bash
make develop          # バックグラウンド起動
make develop ATTACH=1 # バックグラウンド起動後に bash アタッチ
make shell-develop    # develop コンテナに bash でアクセス（未起動なら起動）
```

## シェルアクセス・管理コマンド

```bash
make shell svc=slam      # 実行中コンテナに bash でアクセス
make shell-develop       # develop を起動（未起動なら）して bash アクセス
make logs svc=slam       # ログをフォロー
make ps                  # 起動中コンテナ一覧
make restart svc=slam    # 指定サービスを再起動
make down                # 全サービス停止
make xhost               # xhost +local:docker（GUI 起動前に実行）
make config              # compose 設定の展開確認
```

## GPU override ファイル

| ファイル                  | 対象                                                       |
| ------------------------- | ---------------------------------------------------------- |
| `compose.gpu.nvidia.yaml` | Nvidia GPU（`deploy.resources.reservations.devices` 設定） |
| `compose.gpu.amd.yaml`    | AMD GPU（`/dev/kfd`, `/dev/dri` デバイスマウント）         |

`USE_GPU` の値に応じて Makefile が自動的に `-f` オプションで override ファイルを適用します。  
GPU override の対象サービスは `gazebo-simulation` のみです。

## 依存ファイル収集の仕組み

`make build` 実行時に `docker/collect_deps.sh` が自動実行され、プロジェクト内の `package.xml`・`*.rosinstall`・`*.repos`・`requirements.txt` を `docker/deps/` に収集します。これにより、ソースの変更が rosdep 等のキャッシュレイヤーに影響しなくなります。

追加で収集したいファイルは `docker/deps_extra.txt` にプロジェクトルートからの相対パスで列挙してください。

## 環境変数

`.env` で以下の変数を設定します（`.env.example` を参照）。

| 変数                   | 説明                                                 | デフォルト               |
| ---------------------- | ---------------------------------------------------- | ------------------------ |
| `COMPOSE_PROJECT_NAME` | Compose プロジェクト名（コンテナ名のプレフィックス） | `horiokart`              |
| `USE_GPU`              | GPU の種類。`none` / `nvidia` / `amd`                | `none`                   |
| `SIMULATION`           | シミュレーションモード。`true` / `false`             | `true`                   |
| `USE_RVIZ`             | RViz2 を起動するか。`true` / `false`                 | `true`                   |
| `RVIZ_CONFIG`          | `make rviz` で使用する RViz2 設定ファイルのパス      | （未設定 = デフォルト）  |
| `MAP_PATH`             | マップファイルのディレクトリパス                     | `/root/ros2_data/map`    |
| `ROSBAG_PATH`          | rosbag の保存・再生ディレクトリパス                  | `/root/ros2_data/rosbag` |
| `ROS_DOMAIN_ID`        | ROS2 ドメイン ID                                     | （未設定）               |
| `RMW_IMPLEMENTATION`   | RMW 実装                                             | `rmw_cyclonedds_cpp`     |

