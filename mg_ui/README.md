# mg_ui

MG-01 の状態表示・操作 UI パッケージ群。

## パッケージ構成

| パッケージ          | 説明                                                               |
| ------------------- | ------------------------------------------------------------------ |
| `mg_web_ui`         | React フロントエンド + foxglove_bridge + HTTP 静的配信ノード       |
| `mg_tui`            | ターミナル UI（rclpy + Textual）                                   |
| `mg_system_manager` | SLAM / Navigation の起動・停止を ROS2 サービス経由で実行するノード |

正常性診断は `mg_diagnostics/`（プロジェクトルート）で管理しています。

## 前提条件

- `make slam` または `make navigation` が起動済みであること
- `make diagnostics` と `make system-manager` が起動済みであること（操作系機能を使う場合）

## 通常起動（本番）

```bash
# 本番ビルド（初回または frontend 変更時）
make build svc=web-ui

# ブラウザ UI 起動（foxglove_bridge:8765 + HTTP:8080）
make web-ui

# タブレット / ブラウザからアクセス
# http://<ロボットIP>:8080
```

## 開発モード（フロントエンド変更を即時反映）

```bash
# Vite devサーバー起動（HMR 有効、ポート 5173）
make web-ui-dev

# ブラウザからアクセス
# http://localhost:5173  or  http://<ロボットIP>:5173
```

> foxglove_bridge は `make web-ui` または `make slam` / `make navigation` 側で起動していること。

## TUI（SSH / 制御 PC ターミナル）

```bash
make tui
```

キーバインド:

| キー | 操作                      |
| ---- | ------------------------- |
| `s`  | Waypoint navigation START |
| `x`  | STOP                      |
| `p`  | PAUSE                     |
| `r`  | RESUME                    |
| `m`  | Save map                  |
| `q`  | Quit                      |

## 診断・システム管理

```bash
make diagnostics     # /diagnostics トピックへの正常性診断配信
make system-manager  # SLAM / Nav の起動制御サービス
```

## 将来の点群可視化

foxglove_bridge は起動済みのため、`frontend/src/` にコンポーネントを追加するだけで対応可能です。
`@foxglove/ws-protocol` クライアントは `useFoxgloveClient.ts` に実装済みです。
