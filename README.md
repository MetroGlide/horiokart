# horiokart

自律移動台車ロボット開発プロジェクト。ROS2 (Humble) + Docker による実行環境を提供します。

## 前提条件

- Docker Engine 24.0+
- Docker Compose v2.20+
- GPU を使用する場合:
  - **Nvidia**: [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) のインストールが必要
  - **AMD**: ROCm 対応の `/dev/kfd`, `/dev/dri` が利用可能であること

## セットアップ

```bash
cp .env.example .env
```

`.env` を環境に合わせて編集してください（詳細は[環境変数](#環境変数)を参照）。

## イメージのビルド

```bash
make build-runtime       # slam / navigation / rosbag-replay 用（軽量）
make build-simulation    # gazebo-simulation 用（Gazebo 含む）
make build-develop       # develop 用（全部入り）
make build-all           # 全イメージ一括ビルド
```

## 各サービスの起動

### 実機

```bash
make slam           # SLAM マッピング
make navigation     # 自律ナビゲーション
make rosbag-replay  # rosbag 再生 + nav2
```

### シミュレーション（Gazebo）

```bash
make gazebo-simulation
```

GPU を使用する場合は `.env` の `USE_GPU` を設定するか、コマンドライン引数で上書きできます。

```bash
make gazebo-simulation USE_GPU=nvidia
make gazebo-simulation USE_GPU=amd
```

### 開発

```bash
make develop   # コンテナ起動 + bash アタッチ
```

## 停止

```bash
make down
```

## 環境変数

`.env` で以下の変数を設定します。

| 変数                 | 説明                                     | デフォルト               |
| -------------------- | ---------------------------------------- | ------------------------ |
| `USE_GPU`            | GPU の種類。`none` / `nvidia` / `amd`    | `none`                   |
| `SIMULATION`         | シミュレーションモード。`true` / `false` | `true`                   |
| `USE_RVIZ`           | RViz2 を起動するか。`true` / `false`     | `true`                   |
| `MAP_PATH`           | マップファイルのディレクトリパス         | `/root/ros2_data/map`    |
| `ROSBAG_PATH`        | rosbag の保存・再生ディレクトリパス      | `/root/ros2_data/rosbag` |
| `ROS_DOMAIN_ID`      | ROS2 ドメイン ID                         | （未設定）               |
| `RMW_IMPLEMENTATION` | RMW 実装                                 | `rmw_cyclonedds_cpp`     |

## Docker イメージ構成

```
Dockerfile.base
├── runtime    ← slam / navigation / rosbag-replay
│                  ROS2 基本 + RViz2 + RealSense SDK
├── simulation ← gazebo-simulation
│                  runtime + Gazebo Fortress + rqt
└── develop    ← develop
                   simulation + vim + Qt/ZMQ 開発ライブラリ
```

## GPU override ファイル

| ファイル                         | 対象                                                       |
| -------------------------------- | ---------------------------------------------------------- |
| `docker-compose.gpu.nvidia.yaml` | Nvidia GPU（`deploy.resources.reservations.devices` 設定） |
| `docker-compose.gpu.amd.yaml`    | AMD GPU（`/dev/kfd`, `/dev/dri` デバイスマウント）         |

`USE_GPU` の値に応じて Makefile が自動的に `-f` オプションで override ファイルを適用します。
