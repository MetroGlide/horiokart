# tasks.md — 現状反映版

以下は実装進捗を反映し、実運用（production-ready）を見据えた優先度と次アクションを明確化したタスクリストです。

## 実行ルール（省略）
- タスク ID は T### 形式
- 先行タスクがある場合は Dependencies に TID を列挙

---

## 概要（現状）
- Python ツール群 (`tools/gnss_extract.py`, `tools/gnss_transform.py`, `tools/gnss_match.py`, `tools/gnss_optimize.py`) は PoC 実装と単体テストが存在します。`gnss_match` と `gnss_optimize` はパラメータ化（MAD閾値、loss/f_scale 等）を追加済み。
- C++ 側の `karto_adapter`（PoseGraph エクスポータ）はプロトタイプが存在し JSON を出力できますが、スキーマ準拠（エッジ情報・共分散の出力）とビルド/CI 安定化が必要です。

ポリシー: 実運用に耐えるアルゴリズムを優先
- 優先順位: 使い勝手（CLI、UX）は二次的。まずは共分散を正しく扱うこと、情報行列/重みの導入、IRLS/DCS 等の動的外れ値処理、レバーアーム同時推定などアルゴリズムの堅牢性を優先して実装し、自動テストで精度と安定性を担保する。

KartoAdapter 出力要件（必須）:
- nodes[] は少なくとも `id`, `state_id`, `timestamp`, `pose` (x,y,theta) を含むこと。可能ならノード共分散やノードメタデータも含めること。
- edges[] は `from`/`to`（両方 id と index を含められる場合は両方）、`meas` (dx,dy,dtheta)、`cov` または `info`（edge 共分散／情報行列）、`type` (`scan`|`odom`|`loop`)、及び `source` メタデータを含めること。
- metadata: 出力ツールのバージョン、マップフレーム、投影情報（UTM zone）を含めること。

これらは downstream の共分散対応最適化 (T014 など) と外れ値手法にとって必須です。

---

## タスクリスト（実運用に向けた更新）

T001 — 契約テスト作成
- 状態: completed
- 説明: `contracts/*.schema.json` に基づく pytest ハーネスを実装済み（小規模フィクスチャ含む）。

T002 — C++ PoseGraph prototype（安定化フェーズ）
- 状態: in-progress
- 説明: `karto_adapter` の出力をスキーマ準拠に安定化する。特に `nodes` と `edges` の完全性（id, state_id, pose, timestamp, edge transform, info/covariance）を検証するテストを追加し、colcon/CMake のビルドパスとインクルードを固定する。
- 受け入れ条件: export JSON がスキーマ検証を通過すること。CI でビルドとテストが通ること。
 - 現状メモ: 実装を確認しました。`KartoAdapter`（C++）は BUILD_KARTO_ADAPTER 有効時に `.posegraph` から nodes/edges を抽出し、`relative_pose` と `covariance` を含む JSON を出力します。`ServiceAdapter` は slam_toolbox の serialize サービスを呼ぶ PoC 実装があり、実行時は `/tmp/posegraph_dump.json` を出力する実行バイナリが付属しています。これらは PoC として動作しますが、CMake/CI に組み込んで安定化する必要があります。

T003 — `tools/gnss_extract.py`
- 状態: completed
- 説明: rosbag2 から `sensor_msgs/NavSatFix` を抽出する CLI 実装（rclpy あるいは play-and-capture モード）。

T004 — `tools/gnss_transform.py`
- 状態: completed
- 説明: lat/lon → UTM 変換（`pyproj` 使用）。ゾーン自動判定／明示指定オプションあり。

T005 — `tools/gnss_match.py`（PoC → 強化）
 - 状態: completed (PoC + function-level enhancements)
 - 説明: `match_posegraph_with_gnss` 実装は関数レベルでスプライン補間（SciPy 利用時）と Mahalanobis フィルタ、複数の `weight_mode` を実装済み。MAD フィルタや時間窓 `window` 設定も実装されています。
 - CLI 状況: 現在の CLI exposes `--window`, `--mad-threshold`, `--weight-mode` and `--no-y-filter`, but DOES NOT expose `--interp-method` nor `--mah-threshold` yet（これらは関数引数としては存在する）。運用用 CLI の拡充が必要です。
 - 次の作業: CLI に `--interp-method` / `--mah-threshold` を追加して運用で切り替えられるようにする。また Kalman 補間と cov 出力の追加は未実装（別タスク T013 で計画済み）。
 - 優先タスク更新: T013（Kalman 補間と cov 出力）は優先度を上げ、gnss_match が constraints に `cov` を必ずセットするようにする（GNSS が共分散を提供する前提のため）。

T006 — `tools/gnss_optimize.py`（PoC → 強化）
- 状態: in-progress (PoC exists)
- 説明: GNSS 制約を受け取り SciPy least_squares（ロバスト loss）で XY を最適化する PoC を実装済み。CLI で `--loss` / `--f-scale` を指定可能。
- 次の作業: グラフエッジ（odom/loop closures）を残差に組み込み、GNSS 共分散を重みに反映、エッジとGNSSの重み比を CLI で調整可能にする。
 - 現状メモ: `gnss_optimize.py` は既にエッジ残差を PoC として実装しており、`--edge-weight-scale` CLI オプションを提供しています。ただし GNSS の `cov` を情報行列として使う実装や IRLS/DCS の動的外れ値抑制、レバーアーム同時推定などは未実装です。
 - 次の作業: `constraints[].cov` を読み込んで情報行列を適用する実装（T014）、IRLS/DCS ベースの動的外れ値処理（T015）、及びアンテナオフセットの同時最適化（T016）を進める必要があります。
 - 優先タスク更新: T014/T015/T016 を優先してスケジューリングし、特に `constraints[].cov` を用いる情報行列対応 (T014) を最優先で実装します。

T012 — 時刻同期チェックと自動オフセット推定
- 状態: not-started
- 説明: posegraph と GNSS の時間基準不一致を検出・補正するための機能を追加する。`gnss_match` に `--time-offset` / `--auto-time-offset` を追加し、自動推定（±5s, step=0.1s グリッド探索）で最適シフトを算出して適用する。
- 依存: T005 (gnss_match)、T007 (run_gnss_fusion)
- 期間見積: 0.5–1.5 日
- 受け入れ基準:
	- 合成データで既知オフセットを ±0.1s 精度で復元できること。
	- オフセット適用後のマッチ成功率が向上することを示す回帰テスト。

T013 — Kalman (CVKF) 補間と不確かさ出力
- 状態: not-started
- 説明: `tools/gnss_kalman.py` を追加して Constant-Velocity Kalman Filter による補間を実装。`gnss_match` に `--interp-method kalman` を追加して、推定位置と 2x2 共分散を constraints に含める。
- 依存: T005 (gnss_match), T008 (tests and evaluation)
- 期間見積: 1–3 日
- 受け入れ基準:
	- 合成データ（速度変化あり）で RMSE が線形/スプラインと同等か改善すること。
	- constraints に cov フィールドが含まれているユニットテストが通ること。

T014 — GNSS 共分散を情報行列として最適化に取り込む
- 状態: not-started
- 説明: `constraints[].cov` を用いて情報行列 W = Σ^{-1} を構成し、最適化で残差に適切に反映する。特異行列に対する正則化処理を追加する。
- 依存: T013 (cov 出力), T006 (gnss_optimize)
- 期間見積: 0.5–2 日
- 受け入れ基準:
	- 合成データで cov が小さい観測をより強く尊重することが確認できること。
	- 数値的に特異な cov に対しても安定して動作すること。

T015 — 動的外れ値処理（IRLS / DCS / switchable constraints PoC）
- 状態: not-started
- 説明: 最適化ループ内で残差に応じて制約重みを更新する IRLS、もしくは DCS/switchable constraints の PoC を実装し、外れ値に対する耐性を高める。
- 依存: T006 (gnss_optimize)
- 期間見積: 1–4 日（比較テスト含む）
- 受け入れ基準:
	- 外れ値率 10–30% の合成データで median error が改善すること。
	- 外れ制約の重みが低下するログ/可視化を確認できること。

T016 — レバーアーム（アンテナオフセット）と yaw オフセットの同時推定
- 状態: not-started
- 説明: GNSS アンテナとロボット基準点のオフセット `(tx,ty,theta)` を最適化変数に追加するオプションを実装。オフライン校正モードとして利用できる。
- 依存: T006 (gnss_optimize)
- 期間見積: 1–3 日
- 受け入れ基準:
	- 合成データで注入した既知オフセットを誤差 < 0.2m / 1deg で推定できること。
	- 同時最適化でエッジ情報が大きく破綻しないこと。

T017 — ロバスト損失の自動チューニング（f_scale 自動初期化）
- 状態: not-started
- 説明: 最適化前に残差スケールを推定して `f_scale` を自動設定する機能を追加。`--auto-f-scale` フラグで有効化できる。
- 依存: T006 (gnss_optimize)
- 期間見積: 0.5–1.5 日
- 受け入れ基準:
	- 合成データで自動推定された f_scale が手動最適値に近いこと。
	- 自動化により収束失敗率が低下することを示す回帰テスト。

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
 - A2: `gnss_match` の CLI に `--interp-method` と `--mah-threshold` を追加して運用で切り替え可能にする（T005） — 1–2 hours。関数実装は存在するが CLI 露出が未完。
 - A3: `gnss_optimize` の `edge_weight_scale` は既に CLI 実装済みのため、C++ 側エクスポータとの互換（`from_idx`/`to_idx`, `relative_pose`, `covariance` の安定出力）を確認して統合する（T002/T006 統合タスク） — 1–2 days。
 - A4: `requirements.txt` を CI に反映してユニットテストを CI 上で回す準備（T011） — 30–60 minutes 設定作業。
