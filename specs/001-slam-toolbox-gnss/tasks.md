````markdown
```markdown
# tasks.md — 現状反映版

以下は実装進捗を反映し、実運用（production-ready）を見据えた優先度と次アクションを明確化したタスクリストです。

## 実行ルール（省略）
- タスク ID は T### 形式
- 先行タスクがある場合は Dependencies に TID を列挙

---

## 概要（現状）
- Python ツール群 (`tools/gnss_extract.py`, `tools/gnss_transform.py`, `tools/gnss_match.py`, `tools/gnss_optimize.py`) は PoC 実装と単体テストが存在します。`gnss_match` と `gnss_optimize` はパラメータ化（MAD閾値、loss/f_scale 等）を追加済み。
- C++ 側の `karto_adapter`（PoseGraph エクスポータ）はプロトタイプが存在し JSON を出力できますが、スキーマ準拠（エッジ情報・共分散の出力）とビルド/CI 安定化が必要です。

---

## タスクリスト（実運用に向けた更新）

T001 — 契約テスト作成
- 状態: completed
- 説明: `contracts/*.schema.json` に基づく pytest ハーネスを実装済み（小規模フィクスチャ含む）。

T002 — C++ PoseGraph prototype（安定化フェーズ）
- 状態: in-progress
- 説明: `karto_adapter` の出力をスキーマ準拠に安定化する。特に `nodes` と `edges` の完全性（id, state_id, pose, timestamp, edge transform, info/covariance）を検証するテストを追加し、colcon/CMake のビルドパスとインクルードを固定する。
- 受け入れ条件: export JSON がスキーマ検証を通過すること。CI でビルドとテストが通ること。

T003 — `tools/gnss_extract.py`
- 状態: completed
- 説明: rosbag2 から `sensor_msgs/NavSatFix` を抽出する CLI 実装（rclpy あるいは play-and-capture モード）。

T004 — `tools/gnss_transform.py`
- 状態: completed
- 説明: lat/lon → UTM 変換（`pyproj` 使用）。ゾーン自動判定／明示指定オプションあり。

T005 — `tools/gnss_match.py`（PoC → 強化）
- 状態: completed (PoC)
- 説明: ノード timestamp と GNSS を time-match/補間して制約を生成する機能は実装済み。パラメータ: `window`, `mad_threshold`, `use_y` を追加。
- 次の作業: 重み算出の改善（`--weight-mode`: fixed / cov_trace / hdop / var_interp）、Mahalanobis ベースの外れ値除去、補間の選択肢（spline/kalman）を追加。

T006 — `tools/gnss_optimize.py`（PoC → 強化）
- 状態: in-progress (PoC exists)
- 説明: GNSS 制約を受け取り SciPy least_squares（ロバスト loss）で XY を最適化する PoC を実装済み。CLI で `--loss` / `--f-scale` を指定可能。
- 次の作業: グラフエッジ（odom/loop closures）を残差に組み込み、GNSS 共分散を重みに反映、エッジとGNSSの重み比を CLI で調整可能にする。

T007 — `scripts/run_gnss_fusion.py`（統合スクリプト）
- 状態: in-progress
- 説明: パイプライン（serialize → extract → transform → match → optimize）のラッパーは存在するが、引数整理、堅牢なエラーハンドリング、ログ出力、結果の統計出力（median error 等）を追加する必要がある。

T008 — tests and evaluation (ベンチマーク整備)
- 状態: not-started
- 説明: 複数データセットで median registration error < 1m を目標に評価パイプラインを整備。自動評価、パラメータスイープ、レポート生成を含む。

T009 — Robustify GNSS matching
- 状態: not-started
- 説明: 補間の高度化（スプライン/カルマン）、Mahalanobis による外れ値除去、dt閾値の自動調整、複数 GNSS レコードの集約、欠測時の挙動改善。ユニット/統合テスト追加。

T010 — Harden optimizer for production
- 状態: not-started
- 説明: グラフエッジを残差に組み込み、ロバストロスのチューニング、性能（計算時間・メモリ）プロファイリング。必要なら C++（ceres/g2o）へ移植するための設計。

T011 — CI & dataset benchmarks
- 状態: not-started
- 説明: `requirements`（pyproj, scipy 等）を CI に組込み、公開/社内データセットで夜間バッチ／PR時の簡易評価を行う。

---

## 受け入れ基準（production-ready）
- Posegraph JSON がスキーマ検証に合格すること（自動テスト）。
- GNSS→ノードのマッチングでアウトライアを 95% 以上正しく除外できる（合成データで検証）。
- 最適化後の median registration error が指定評価セットで 1m 未満であること。
- パイプラインは CLI で再現可能でログ・統計を出力すること。

---

## 優先度付きネクストアクション（短期 → 中期）

短期（今日〜数日）
- A1: `karto_adapter` の JSON を schema 検証する自動テストを追加（T002） — half day。これにより C++ 出力の品質ゲートを作る。
- A2: `gnss_match` に `--weight-mode` を追加（fixed / cov_trace / hdop / var_interp）し、ユニットテストを追加（T005） — 1–2 hours。GNSS 共分散を重みに反映することで精度が上がる。
- A3: `gnss_optimize` に `--edge-weight-scale` を追加してエッジ/観測の比率を CLI から調整可能にする（T006） — 30–60 minutes。
- A4: `requirements.txt` を CI に反映してユニットテストを CI 上で回す準備（T011） — 30–60 minutes 設定作業。

中期（数日〜1〜2 週間）
- B1: グラフエッジを残差に組み込み、同時最適化を検証（T010） — 3–7 days（PoC→安定化）。
- B2: 補間手法をカルマン/スプラインに切替可能にして外れ値検出を Mahalanobis ベースへ移行（T009） — 2–4 days。
- B3: 評価パイプライン構築（自動化されたパラメータスイープとレポート）を実装（T008/T011） — 3–5 days。

長期（移行）
- C1: 大規模データ用に最適化器を C++（ceres/g2o）へ移植し、runtime とメモリを改善。

---

次に私がやれる作業（選択してください）:
- 1) A1: `karto_adapter` の JSON を schema 検証する自動テストを作成します（推奨、最優先）。
- 2) A2: `gnss_match` の `--weight-mode` を実装してユニットテストを追加します（即効性あり）。
- 3) A3: `gnss_optimize` のエッジ重みスケールを実装します（簡単）。
- 4) `scripts/run_gnss_fusion.py` の引数整理とログ/統計出力の追加（中程度の作業）。

選択 (例: "1 and 2" や "2" ) を教えてください。選ばれたら、即座に実装してテストまで実行します。
```
