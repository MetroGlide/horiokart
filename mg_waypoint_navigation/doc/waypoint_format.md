# ウェイポイント YAML フォーマット v2.0

## 概要

`mg_waypoint_navigation` で使用するウェイポイントファイルの仕様。
`version: "2.0"` キーで識別される。旧フォーマット(v1)は後方互換性で読み込み可能。

---

## v2.0 フォーマット仕様

```yaml
version: "2.0"

# 全ウェイポイントに適用するデフォルト値 (省略可)
defaults:
  reach_tolerance: 0.5       # ゴール到達判定半径 [m]
  through_tolerance: 3.0     # 通過点の到達判定半径 [m]
  is_through_point: true     # true=通過点, false=停止点

waypoints:
  - index: 0
    pose:
      position: {x: 1.0, y: 2.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
    navigation:               # このウェイポイント固有の設定 (省略=defaults適用)
      is_through_point: false
    on_reached_actions: []    # 到達時アクションリスト (省略可)
```

---

## on_reached_actions 型一覧

### type: service — サービス呼び出し

```yaml
- type: service
  service: /front_lidar_publish_controller_node/change_publish_state
  srv_module: std_srvs.srv
  srv_class: SetBool
  request:
    data: false
```

| フィールド   | 型     | 説明                                   |
| ------------ | ------ | -------------------------------------- |
| `service`    | string | サービスパス                           |
| `srv_module` | string | Python モジュール (例: `std_srvs.srv`) |
| `srv_class`  | string | サービス型クラス名 (例: `SetBool`)     |
| `request`    | dict   | リクエストフィールドと値               |

### type: publish — トピックパブリッシュ

```yaml
- type: publish
  topic: /gnss_odometry_node/select_static_transform
  msg_module: std_msgs.msg
  msg_class: String
  data:
    data: "transform_label_A"
```

| フィールド   | 型     | 説明                                   |
| ------------ | ------ | -------------------------------------- |
| `topic`      | string | トピックパス                           |
| `msg_module` | string | Python モジュール (例: `std_msgs.msg`) |
| `msg_class`  | string | メッセージ型クラス名 (例: `String`)    |
| `data`       | dict   | メッセージフィールドと値               |

### type: load_map — マップロード

```yaml
- type: load_map
  localization: /root/ros2_data/map/area1/localization.yaml
  planning: /root/ros2_data/map/area1/planning.yaml
```

| フィールド     | 型     | 説明                          |
| -------------- | ------ | ----------------------------- |
| `localization` | string | 測位マップ YAML パス (省略可) |
| `planning`     | string | 計画マップ YAML パス (省略可) |

### type: amcl_reset — AMCL リセット

```yaml
- type: amcl_reset
```

`/reinitialize_global_localization` (std_srvs/Empty) を呼び出す。

### type: wait — 待機

```yaml
- type: wait
  countdown_ms: 3000
```

| フィールド     | 型  | デフォルト | 説明          |
| -------------- | --- | ---------- | ------------- |
| `countdown_ms` | int | 3000       | 待機時間 [ms] |

### type: wait_trigger — 外部トリガー待ち

```yaml
- type: wait_trigger
```

到達時アクションをすべて実行した後、FSM を IDLE 状態に移行させ外部からの `~/start` サービス呼び出しを待つ。
`~/start` リクエストの `countdown_ms` でカウントダウン後に次のウェイポイントへ進む。

フィールドなし。

---

## 実際の設定例

```yaml
version: "2.0"
defaults:
  reach_tolerance: 0.8
  is_through_point: true

waypoints:
  - index: 0
    pose:
      position: {x: 0.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    navigation:
      is_through_point: true

  - index: 1
    pose:
      position: {x: 10.0, y: 0.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    navigation:
      is_through_point: false
    on_reached_actions:
      - type: service
        service: /front_lidar_publish_controller_node/change_publish_state
        srv_module: std_srvs.srv
        srv_class: SetBool
        request: {data: false}
      - type: load_map
        localization: /root/ros2_data/map/area2/localization.yaml
        planning: /root/ros2_data/map/area2/planning.yaml
      - type: wait
        countdown_ms: 3000
      - type: amcl_reset
      - type: service
        service: /front_lidar_publish_controller_node/change_publish_state
        srv_module: std_srvs.srv
        srv_class: SetBool
        request: {data: true}

  - index: 2
    pose:
      position: {x: 20.0, y: 5.0, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
    on_reached_actions:
      - type: publish
        topic: /gnss_odometry_node/select_static_transform
        msg_module: std_msgs.msg
        msg_class: String
        data: {data: "label_B"}
```

---

## v1 フォーマットとの差分

| 項目                  | v1                                 | v2.0                                     |
| --------------------- | ---------------------------------- | ---------------------------------------- |
| ファイル構造          | リスト直接                         | `{version, defaults, waypoints}`         |
| アクション定義        | `on_reached_action: [string]` enum | `on_reached_actions: [{type, ...}]` dict |
| reach_tolerance       | トップレベルフィールド             | `navigation.reach_tolerance`             |
| is_through_point      | トップレベルフィールド             | `navigation.is_through_point`            |
| gnss_transform_label  | トップレベルフィールド             | `publish` アクションで代替               |
| localization_map_yaml | トップレベルフィールド             | `load_map` アクションで代替              |

---

## v1 → v2 マイグレーション

```bash
ros2 run mg_waypoint_navigation migrate_waypoints.py input.yaml output_v2.yaml
```

出力ファイル名を省略した場合は `input_v2.yaml` として出力される。

### v1 アクション文字列と v2 の対応

| v1 文字列                     | v2 type    | 備考                                                                   |
| ----------------------------- | ---------- | ---------------------------------------------------------------------- |
| `front_lidar_off`             | `service`  | `/front_lidar_publish_controller_node/change_publish_state` data=false |
| `front_lidar_on`              | `service`  | 同 data=true                                                           |
| `amcl_on`                     | `service`  | `/amcl/enable` data=true                                               |
| `amcl_off`                    | `service`  | `/amcl/enable` data=false                                              |
| `gps_on`                      | `service`  | `/gnss_odometry_node/change_publish_state` data=true                   |
| `gps_off`                     | `service`  | `/gnss_odometry_node/change_publish_state` data=false                  |
| `reload_map`                  | `load_map` | `localization_map_yaml` / `planning_map_yaml` を引き継ぎ               |
| `wait_trigger`                | `wait`     | `countdown_ms: 0`                                                      |
| `select_gnss_transform_label` | `publish`  | `gnss_transform_label` フィールドを data.data に変換                   |
