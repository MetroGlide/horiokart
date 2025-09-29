# GNSS → slam_toolbox PoseGraph 統合ドキュメント

目的
- slam_toolbox の 2D posegraph に対して、GNSS（sensor_msgs/NavSatFix）由来の fix を拘束（constraints）として追加し、オフラインで地球座標系に整列するワークフローを定義する。
- 非侵襲的に（slam_toolbox のコア変更なしで）アダプタと外部ツール群で処理を完結させる。

対象リポジトリ
- パス: `horiokart_slam/`
- 主要ツール: `tools/gnss_extract.py`, `tools/gnss_transform.py`, `tools/gnss_match.py`, `tools/gnss_optimize.py`

前提と制約
- slam_toolbox の serialize 機能（ServiceAdapter または KartoAdapter 経由）でノードに `timestamp` を含む posegraph JSON を出力できること。
- GNSS データは `sensor_msgs/NavSatFix` から抽出して経緯度を UTM (x,y) に変換する前提（`gnss_transform.py` を使用）。
- 現実装は Python + SciPy を用いたオフライン最適化 PoC。主なアルゴリズムは以下に記載の通りで、近似的な Kalman 補間（CVKF）と共分散→情報行列変換（T014）を含みます。将来的に性能向上のため C++ 実装 (ceres/g2o) を検討します。

全体処理フロー（ステップ）
1. PoseGraph の出力（serialize）
   - slam_toolbox のサービス `/slam_toolbox/serialize_map` または Karto のアダプタで posegraph を JSON 形式でエクスポート。
   - 必須フィールド（スキーマ推奨）: nodes[].id (任意), nodes[].state_id, nodes[].pose [x,y,yaw], nodes[].timestamp (sec or float)
   - 推奨: edges[] に `from_idx`/`to_idx` と相対変位 `dx`,`dy` を含めると optimizer と親和性が高い。

2. GNSS 抽出 (`tools/gnss_extract.py`)
   - rosbag2 などから `sensor_msgs/NavSatFix` を抽出して JSON/CSV を作る。出力例: `records: [{time: <sec>, lat:, lon:, accuracy:, hdop:, cov: [...]}, ...]`

3. GNSS 変換 (`tools/gnss_transform.py`)
   - `pyproj` を使い lat/lon → UTM に変換。出力は UTM 座標 `x`, `y` と時間 `time` を持つ JSON を作成。
   - オプション: 明示ゾーン指定、経度帯の強制指定など。

4. 時間一致と補間 (`tools/gnss_match.py`)
 - 概要: 各ノードの `timestamp` に対して近傍 GNSS を time-window（デフォルト `window=2.0` 秒）で探索し、補間して `constraints` を生成する。
 - 補間方式（`--interp-method`）: `linear`（デフォルト）、`spline`（SciPy の UnivariateSpline、3点以上）、`kalman`（Constant-Velocity Kalman Filter, CVKF）。
   - 現状実装: `kalman` は CVKF の簡易実装を提供し、任意時刻での状態（x,y）と 2x2 共分散を返す。
   - CVKF の内部デフォルトパラメータ（PoC）: position process noise `q_pos=0.1`、velocity process noise `q_vel=1e-2`、観測ノイズベース `R_base=1.0`。将来的に CLI で露出予定。
 - 外れ値除去:
   - MAD フィルタ（デフォルト `mad_threshold=3.5`）
   - Mahalanobis フィルタ（`--mah-threshold`、デフォルト None。利用時の目安 `5.0`）
 - 重み付けモード (`--weight-mode`, デフォルト `var_interp`): `fixed` | `var_interp` | `cov_trace` | `hdop`
   - `var_interp`: 補間時の不確かさを近似して分散に基づく重み (1/var)
   - `cov_trace`: GNSS が提供する 2x2 共分散（row-major 4 要素）を使い trace ベースで重み化
   - `hdop`: HDOP に逆比例
 - 時間オフセット:
   - CLI オプション: `--time-offset`（固定シフト） と `--auto-time-offset`（グリッド探索）をサポート。
   - `--auto-time-offset` の現実装: 範囲 `-5.0..+5.0` s、ステップ `0.1` s でオフセットを適用したうえで最も多くマッチしたオフセットを採用（評価指標は現状マッチ数）。
 - 出力: `constraints` JSON（各要素に `node_idx`, `x`, `y`, `weight`, `source_time`, `dt`、可能なら `cov` を含む）

  アルゴリズム詳細とパラメータ
  - 前処理:
    - GNSS レコードは時刻順にソートする。欠損値（x,yが無い）や極端な精度劣化（accuracy/hdop が大きすぎる）は除外する。
    - 推奨デフォルトパラメータ: time window `window = 2.0` 秒、MAD 閾値 `mad_threshold = 3.5`、Mahalanobis 閾値 `mah_threshold = 5.0`（必要に応じて調整）。

  - 補間手順（ノード t に対する GNSS 推定）:
    1. ノード時刻 t に対し、時刻差が `<= window` の GNSS サンプルを検索する。候補サンプルが 0 個の場合はそのノードの GNSS 制約を生成しない（skip）。
    2. 候補サンプルが 1 個ならそのサンプルをそのまま採用（x,y を使用）。
    3. 候補サンプルが 2 個以上なら補間:
       - linear: 直近の前後サンプルを使い線形補間。alpha = (t - t0)/(t1 - t0)。
       - spline: 3 点以上が得られ、SciPy が利用可能な場合は UnivariateSpline(times, xs/ys, s=0) を構築し評価（滑らかな補間）。
       - fallback: spline が失敗した場合は線形に落とす。

  - 外れ値除去:
    - MAD フィルタ: 各座標軸に対して median と MAD を用いて外れ値を除去（式: MAD = median(|xi - median(x)|)）。閾値は `k * MAD`（k=mad_threshold）を使う。
    - Mahalanobis フィルタ: 補間後の集合に対し共分散行列を計算し、平均からの Mahalanobis 距離 d が `d > mah_threshold` の点を除去する。共分散が特異な場合は小さい正則化項を追加して安定化する。

  - 重み付け (weight_mode の実装指針):
    - 固定 (fixed): weight = 1.0
    - var_interp: 補間誤差分散を推定し weight = 1.0 / (var + eps)。線形補間では var を両サンプルの精度（accuracy^2）で近似。
    - cov_trace: GNSS が提供する 2x2 共分散行列の trace を用い、weight = 1.0 / (trace + eps)
    - hdop: weight = 1.0 / (hdop + eps)
    - 実装ノート: eps = 1e-6 を追加してゼロ割を避ける。重みは最終的に sqrt(weight) を残差に乗じることを想定（scaling の観察により f_scale で微調整）。

  - 出力フォーマット（constraints の例）:
    - { node_idx: 12, node_id: 345, time: 1590000000.5, x: 123456.78, y: 432100.12, weight: 0.8 }

  -- エッジケース:
    - GNSS サンプルが疎でノードに対してサンプルが無い場合はスキップ。後で最適化中にエッジを基に位置が補間されるようにする。
    - 大気・マルチパスなどにより極端に大きな共分散や HDOP が報告される場合はフィルタで早期除外する。

5. 最適化 (`tools/gnss_optimize.py`)
 - 入力: posegraph JSON（nodes[], edges[]）、constraints JSON
 - 変数: 各 node の XY（yaw は固定）を最適化変数とする（2D 最適化）
 - 目的関数:
   - GNSS residuals: (node_xy - gnss_xy) * sqrt(weight) を残差に含める
   - Edge residuals (PoC): ((xb-xa) - measured_dx, (yb-ya) - measured_dy) を追加し、`edge_weight_scale` で重みを調整
 - ロバスト損失: `least_squares(..., loss='huber', f_scale=...)` を利用（CLI で切替可）
 - 出力: 最適化後の posegraph JSON（nodes[].pose が更新される）と最適化レポート

 重要な更新（現時点の実装）:
  - `constraints[].cov`（2x2 共分散、row-major 4 要素）を読み取り、まず小さな正則化 `cov_regularization`（CLI: `--cov-regularization`, デフォルト `1e-6`）を加えてから逆行列（情報行列）に変換する。
  - 得られた情報行列は残差に適用するために sqrt-information（Cholesky が失敗した場合は固有分解で sqrt を取るフォールバック）を計算し、残差に掛けて重み付けを行う実装を含む。
  - これにより単純な `weight` 値ではなく、共分散情報を厳密に扱うことが可能（PoC 実装の要旨）。

  アルゴリズム詳細とパラメータ
  - 変数とマッピング:
    - 最適化変数は各ノードの XY (
      x = [x0, y0, x1, y1, ..., xN-1, yN-1]
      ) の一次元配列。
    - document 内 `node_idx` は nodes[] の配列インデックスに対応することを前提にする。もし C++ 側が異なる ID を出力する場合は `id->index` マップを作成して変換する。

  - 残差設計:
    - GNSS 残差:
      r_g = sqrt(w) * (p_node - p_gnss)
      ここで p_node = [x_i, y_i], p_gnss = [x_g, y_g], w は `constraints` の weight。残差は [r_gx, r_gy] を最小化対象に加える。
    - エッジ残差（PoC 実装）:
      r_e = edge_weight_scale * ( (p_b - p_a) - p_meas )
      ここで p_meas は edge に含まれる既知の相対変位 [dx, dy]。edge_weight_scale により GNSS とエッジの相対寄与を調整。
    - まとめて残差ベクトル r を作り、scipy.optimize.least_squares で最小化する。

  - ロバスト損失とスケーリング:
    - 推奨: loss='huber', f_scale=1.0（デフォルト）。実データでは `f_scale` を 0.5〜2.0 でスイープしてロバスト性を確認。
    - `edge_weight_scale` の初期候補: 0.1〜10。大きくするとエッジ形状を優先、小さくすると GNSS に追従。

  - 初期値と固定パラメータ:
    - 初期値は元の posegraph の nodes[].pose の XY を利用する。
    - Yaw（theta）は現状固定（2D XY 最適化の簡易化）。必要なら後段で yaw を含める拡張を行う。

  - 実装ノートと安定化:
    - GNSS 共分散を利用できる場合、weight に直接反映する（weight = 1.0 / cov_trace など）。より厳密には情報行列を残差に掛けることで最尤解に近づく。
    - エッジがノード ID を参照する場合は、事前に `id->index` マップを作る。PoC は `from_idx`/`to_idx`（インデックス）を期待する。
    - 共分散行列が不良（非正定）な場合は小さい正則化項を diag に加える。
    - 大規模データでは SciPy の最適化が遅くなるため、収束・性能確認後に C++ 実装 (ceres/g2o) への移行を検討する。

  - 収束判定と出力:
    - least_squares の結果オブジェクト `res` の `success` と `message` を記録する。
    - 最適化結果は nodes[].pose の XY を更新して出力。yaw は元値を保持。

  - チューニングのガイドライン（経験則）:
    - GNSS が高品質（HDOP < 0.8、accuracy < 2m）の場合: edge_weight_scale を 0.1〜1.0 に設定して GNSS を優先。
    - GNSS が低品質（HDOP > 1.5、accuracy > 5m）の場合: edge_weight_scale を 1.0〜10 に設定して odom/loop を優先。
    - MAD や Mahalanobis の閾値はセッション毎に異なるため、まず合成データで 3.5（MAD）/5.0（Mahalanobis）から試し、必要に応じて自動推定ロジックを導入する。

  -- デバッグと可視化:
    - 最適化前後でノード位置をプロットし、GNSS 制約点との距離ヒストグラム（median, RMSE）を出す。
    - エッジ残差の分布と GNSS 残差の分布を別々に可視化して、どちらが最適化を支配しているかを確認する。

データ構造（JSON スキーマの簡易説明）
- posegraph.json
  - nodes: [{ id?: int, state_id?: int, pose: [x,y,yaw], timestamp: float }, ...]
  - edges: [{ from_idx: int, to_idx: int, dx?: float, dy?: float, info?: [4x?], cov?: [...] }, ...]

- gnss_records.json
  - records: [{ time: float, x: float, y: float, hdop?: float, cov?: [4 elems row-major], accuracy?: float }, ...]

- constraints.json
  - constraints: [{ node_idx: int, node_id?: int, time: float, x: float, y: float, weight: float }, ...]

CLI の例
- PoseGraph エクスポート (slam_toolbox サーバ側)
  (このコマンドは ROS2 ノード環境で実行)
  # service call を実行して posegraph.json を保存

- GNSS 変換
  python3 tools/gnss_transform.py --in gnss_raw.json --out gnss_utm.json

- GNSS マッチング
  python3 tools/gnss_match.py --posegraph posegraph.json --gnss gnss_utm.json --out constraints.json --interp-method spline --mah-threshold 5.0

- 最適化
  python3 tools/gnss_optimize.py --posegraph posegraph.json --constraints constraints.json --out posegraph_opt.json --loss huber --f-scale 1.0 --edge-weight-scale 1.0


運用上の注意点
- GNSS の時間同期: posegraph の timestamp と GNSS の time が同じ時間基準 (epoch 秒) であることを確認すること。
- GNSS 密度が非常に低い場合、補間が不安定になり得る。dt 閾値でマッチしないノードはスキップする方が安全。
- GNSS の共分散情報がない場合は `hdop` や fixed weight を利用して重み付けを行う。


参考
- `horiokart_slam/tools/` 配下の実装を参照。具体的な関数: `match_posegraph_with_gnss`, `optimize_posegraph`.

---
