## horiokart_obstacle_detector_3d 実装設計

このドキュメントは、ROS 2 Humble をベースに屋外差動二輪ロボットの前方障害物検知パッケージ `horiokart_obstacle_detector_3d` の実装設計をまとめたものです。

目的:
- Realsense D415（斜め下向きに正面取付）から得られる PointCloud2 を用いて、走行中の路面（base_linkのある平面）と障害物を区別し、障害物点群を PointCloud2 と 2D 投影（LaserScan 相当）で配信する。
- 屋外の凹凸や傾斜を考慮し、走行可能な傾斜面を誤って障害物扱いしないこと。
- 入出力周波数 10Hz 以上を目指す（処理最適化を前提）。

## 要件（要約）
- ROS 2 Humble
- 入力: sensor_msgs/PointCloud2（Realsense D415）
- 出力: sensor_msgs/PointCloud2（障害物点群）, sensor_msgs/LaserScan（障害物の2D投影）
- Realsense の取り付け姿勢は既知（TF で base_link へ変換）
- 屋外の路面は完全な平面ではないため、局所的凹凸を許容する地面モデルを採用
- 走行可能な傾斜は地面と判定（誤検知の回避）
- コア処理（ヘッドレス）と ROS インターフェイス（ノード）を分離

## 成功基準
- 正常な屋外シーン（小さな段差、坂、草地、石が散在）で地面と障害物が妥当に分離できる
- 入出力周波数 >= 10Hz（D415 のデータ量を想定）
- 障害物検出の偽陽性（地面を障害物と判定）を低く抑える

## パッケージ構成（提案）

- horiokart_obstacle_detector_3d/
  - CMakeLists.txt
  - package.xml
  - src/           — ROS ノード実装（rclcpp）: TF/トピック/パラメータ管理、メッセージ入出力
  - include/       — 公開ヘッダ: コアライブラリの API 宣言（Doxygen コメント推奨）
  - src_core/      — コア処理実装（GridHeightMap, GroundSeparator, ClusterDetector, ScanProjector などのヘッドレス実装）
  - config/        — デフォルトパラメータ（YAML）
  - launch/        — 起動例（launch ファイル）
  - doc/           — 設計・使用法・パラメータ説明（本ファイルや追加ドキュメント）
  - test/          — 単体（gtest）・統合（rosbag）テスト
  - rviz/          — RViz 設定（中間出力の可視化用、任意）
  - tools/         — 開発用スクリプト（可視化・ログ解析など、任意）

設計方針:
- コア処理は PCL と Eigen を使う C++ 実装（ヘッドレス）。ROS 依存は最小限。
- ノード側で TF、パラメータ、topic/subscription/publish を扱う。

## データフロー（高レベル）
1. subscribe: sensor_msgs/PointCloud2（カメラフレーム）
2. TF で base_link フレームへ変換
3. ROI（x,y,z 範囲）でクロップ
4. ダウンサンプル（VoxelGrid）とノイズ除去（Radius/Statistical）
5. GridHeightMap を作成（2D グリッドに高さ中央値を格納）
6. ローカル勾配（傾斜）を計算し、地面セルを判定
7. 地面点を除去し、非地面点群を生成
8. クラスタリング（Euclidean）で物体を抽出
9. クラスタごとに体積・高さの基準で障害物を判定
10. 障害物点群を PointCloud2 で publish
11. 2D 投影（角度バケット）で LaserScan 相当を publish

## 主要アルゴリズム詳細

### 概要と実装方針
本節では、アルゴリズムの詳細手順、数式、疑似コード、代表的パラメータ、計算量（大まか）を提示します。実装は C++/PCL を想定し、各ステップは独立した関数（ユニットテスト可能）として分離します。

### 1) ROI 切り出し（Crop）
目的: 処理範囲を限定して計算量を削減する。

手順:
- 点群を base_link へ変換（tf2）。
- 各点 p = (x,y,z) について条件 x_min <= x <= x_max, y_min <= y <= y_max, z_min <= z <= z_max を満たす点のみ残す。

疑似コード:
```
for p in cloud:
  if in_roi(p): keep p
```

計算量: O(N), N は入力点数。

代表パラメータ: x_min=0.1, x_max=5.0, y_min=-1.5, y_max=1.5, z_min=-1.0, z_max=2.0

### 2) ダウンサンプルとノイズ除去
目的: 点数削減とスパイク除去。屋外では反射やセンサノイズが多い。

手順:
- VoxelGrid: 点群をセルに分け、各セル内の代表点を保存（中心または平均）。leaf_size = l。
- RadiusOutlierRemoval: 各点について r 半径内の点数が min_neighbors 未満なら削除。

疑似コード:
```
cloud_ds = voxel_grid_downsample(cloud, leaf_size=l)
cloud_clean = radius_outlier_removal(cloud_ds, radius=r, min_neighbors=k)
```

計算量: VoxelGrid は O(N) と近似可能。Radius removal は kd-tree 検索で O(N log N)（近傍探索に依存）。

代表パラメータ: leaf_size=0.02–0.05 m, radius=0.05–0.15 m, min_neighbors=2–5

### 3) GridHeightMap の構築（主要アイデア）
目的: 局所的な路面高さをロバストに推定し、凹凸や小石を吸収して「地面」を表す高さマップを作る。

アルゴリズム:
- ROI 平面（x,y）をセル幅 s で分割し、セルインデックス (i,j) を計算:
  i = floor((x - x_min) / s), j = floor((y - y_min) / s)
- 各セルに観測された z 値を集める。セルの代表高さ z_cell は中央値 (median) を採用:
  z_cell = median({ z_p | p in cell })
- 欠損セル（観測点がない）には近傍セルの補間（例: 隣接平均や線形補間）を適用。

数式:
- セル集合 C = {c_ij}
- 各 c_ij の高さ h_ij = median({ z_p | p in c_ij })

近傍補間（簡易）:
h_ij = mean({ h_mn | (m,n) in N(i,j) and cell not empty })

計算量: 点群をセルに振り分ける操作は O(M)（M はダウンサンプル後の点数）。中央値計算は各セルの集計に依存し、全体で O(M log S)（S はセル内点数）と見積もれるが実装上は線形平均で近似可能。

代表パラメータ: cell_size s = 0.05–0.1 m

実装注意:
- メモリ: グリッド行数 = ceil((x_max-x_min)/s)、列数 = ceil((y_max-y_min)/s)。過度に小さい s はメモリと計算を増やす。

### GridHeightMap の欠損補間と信頼度（confidence）設計
Depth カメラ（Realsense D415 等）では観測欠損（穴）や視野角依存のノイズが頻発するため、単純な欠損補間は危険です。以下は実装指針で、実装者がそのまま使えるデータ構造、アルゴリズム、パラメータ例、疑似コードを示します。

1) セル構造（推奨）
- 各セルに保持する情報:
  - `bool has_observation`  // 現フレームで観測があるか
  - `double height_median`  // 代表高さ（中央値または trimmed mean）
  - `double height_mean`
  - `double height_variance`
  - `int obs_count`         // 直近フレームでの観測数
  - `double confidence`     // 0..1 の信頼度スコア
  - `rclcpp::Time last_observed` // 最終観測時刻（タイムアウト管理）

2) セルへの集約方針
- 各点 p(x,y,z) をセル (i,j) に割り当て、セル内では z 値のリスト（または固定長バッファ）を集める。
- 代表値は可能なら `median(z_list)`（外れ値に強い）。コストが高い場合は trimmed-mean を代替。
- 最低観測数 `min_obs_per_cell_for_confident_median`（例 3）を満たさないセルは低信頼扱いとする。

3) 信頼度算出（cell.confidence）
- 信頼度は観測数・分散・平均距離・視線角などから複合的に算出。正規化して 0..1 にする。
- 例（正規化合成）:
  - count_score = clamp((obs_count - n_min)/(n_max - n_min), 0, 1)
  - variance_score = exp(-k_var * variance)
  - distance_score = exp(-k_dist * (mean_distance - d0))
  - angle_score = max(0.0, cos(mean_view_angle))^p
  - confidence = w1*count_score + w2*variance_score + w3*distance_score + w4*angle_score
- デフォルト係数例: n_min=1, n_max=10, k_var=10.0, k_dist=0.5, w1=0.4,w2=0.3,w3=0.2,w4=0.1

4) 欠損セルの分類（小穴 vs 大穴）
- 空セルを連結成分解析で検出し、面積や直径で分類:
  - 補間可 (small hole) if hole_area <= max_interp_area（例 0.5 m^2）かつ max_diameter_cells <= d_max
  - 補間不可 (large hole) は unknown として扱う（confidence=0）

5) 空間補間（小穴向け）
- 推奨: 重み付き逆距離補間 (IDW) にセルの信頼度を乗じる。
  - neighbors = cells within radius_interp (cells)
  - weight_k = conf_k * (1 / (dist(cell,cell_k) + eps)^p)
  - h_interp = sum(weight_k * h_k) / sum(weight_k)
  - conf_interp = min(mean(conf_k) * interp_alpha, conf_max)
- パラメータ例: radius_interp=3 cells, p=2.0, interp_alpha=0.6, conf_min_for_using_as_ground=0.5

6) 大穴（補間不可）の扱い
- unknown セルは confidence = 0 として扱い、地面判定には用いない。
- 安全措置: unknown 領域上で高い非地面点が観測されれば障害物候補として扱うが、単発点はノイズとして無視するなど保守的に扱う。

7) 時間的融合（短期安定化）
- 各セルに短い履歴（last_k_heights）か指数移動平均 (EMA) を持たせる:
  - h_t = alpha * h_obs + (1-alpha) * h_{t-1}
  - conf_t = alpha_conf * conf_obs + (1-alpha_conf) * conf_{t-1}
- 例: temporal_alpha_height = 0.3, temporal_alpha_conf = 0.4
- 古い観測は `observation_timeout`（例 0.5s）で無効化する。

8) 地面判定時の保守的ルール
- セルを地面として使う条件の例:
  - is_ground_cell_by_slope(i,j) == true
  - AND cell.confidence >= conf_min_for_ground（例 0.5）
- 補間セルは confidence が低くなるため自動的に除外され、誤補間による誤判定を防ぐ。

9) 観測ジオメトリの考慮
- 各点の視線角（カメラ光軸と点ベクトルの角度）や距離を集計し、斜め・遠距離観測は confidence を下げる。Depth カメラの特性に基づく係数調整を推奨。

10) 代表パラメータ（推奨初期値）
- cell_size = 0.05 m
- min_obs_per_cell_for_confident_median = 3
- radius_interp_cells = 3
- interp_power_p = 2.0
- interp_alpha = 0.6
- max_interp_area_m2 = 0.5
- conf_min_for_ground = 0.5
- temporal_alpha_height = 0.3
- temporal_alpha_conf = 0.4
- observation_timeout = 0.5 s

11) 疑似コード（要約）
```
build_grid(cloud):
  clear per-cell buffers
  for p in cloud:
    (i,j) = xy_to_cell(p.x,p.y)
    cell_buffers[i][j].push(p.z, p.distance, view_angle)

  for each cell:
    if buffer empty: mark has_observation=false; continue
    compute median/mean/variance, obs_count, mean_distance, mean_view_angle
    cell.confidence = compute_confidence(...)
    apply temporal fusion with previous state

interpolate_holes(grid):
  find connected empty regions
  for hole in holes:
    if area(hole) <= max_interp_area:
      for each cell in hole:
        neighbors = get_neighbors_within_radius(cell, radius_interp)
        weights = [ conf_k * (1/(dist+eps)^p) for k in neighbors ]
        if sum(weights)==0: continue
        cell.height_median = sum(weights * neighbors.height)/sum(weights)
        cell.confidence = min(mean(neighbors.conf), interp_alpha)
    else:
      mark all cells in hole as unknown (confidence=0)

use_grid_for_ground_removal(cloud):
  for p in cloud:
    (i,j) = xy_to_cell(p)
    if cell.exists and cell.confidence >= conf_min_for_ground and is_ground_cell(i,j):
      label p as ground if |p.z - cell.height_median| <= ground_max_distance
    else:
      label p as non-ground (or unknown -> conservative)
```

12) テスト & 可視化
- 合成データ（平坦/坂/段差/穴）で自動単体テストを作成。
- rosbag 実験で `confidence map` と `interpolated cells` を RViz で可視化し、補間回数・補間面積・cell_coverage_ratio をログで記録。

13) 実装トレードオフ注意
- 中央値は堅牢だが計算負荷が高い。セルあたり点数が多い場合は trimmed-mean を検討。
- 補間は小穴のみ許容し、大穴は unknown として安全優先で扱う。


### 4) ローカル法線・傾斜算出
目的: 各セルの地表の向きを求めて、傾斜角で地面/非地面を分離する指標とする。

方法A（法線推定）:
- 各セルに属する点集合から PCA を行い、最小分散方向を法線 n = (n_x, n_y, n_z) として得る。
- 傾斜角 θ = acos(|n ・ z_unit|) , z_unit = (0,0,1)

方法B（有限差分）:
- 高さマップ h_ij の差分で勾配を近似:
  g_x = (h_{i+1,j} - h_{i-1,j}) / (2s)
  g_y = (h_{i,j+1} - h_{i,j-1}) / (2s)
  傾斜角 θ = atan2(sqrt(g_x^2 + g_y^2), 1)

計算量: セル数を G とすると O(G)（差分）または PCA をセル内点数に応じて計算するため追加コスト。

代表パラメータ: PCA を使う場合はセル当たり最低点数（例 3–5 点）を要求。

### 5) 地面セル判定ルール
目的: 傾斜角・高さ変動・前方連続性に基づいて地面セルを判定し、局所的な凸凹を容認する。

判定条件（例）:
- θ <= slope_threshold (deg)  // 傾斜が緩やか
- local_height_variation = max_neighbor(h) - min_neighbor(h) <= height_var_threshold
- 距離閾値: セルの高さが期待地面高さ（例えば vehicle_wheel_height 参照）から大きく外れていない

連続性チェック:
- 前方方向（x 軸正方向）に対して走行可能セルが一定幅（例 幅 >= w_min）連続しているかを確認。これにより一時的な穴や突起で誤判定するのを防ぐ。

推薦値:
- slope_threshold = 15°、height_var_threshold = 0.12 m、w_min = 0.3 m

### 地面判定：複合指標（傾斜だけでは不十分）
傾斜（slope）の閾値のみで地面判定を行うと、縁石や段差、狭い斜面、車体サイズに依存する可通過性などで誤判定が発生します。以下は実装で推奨する多指標の組合せと実装手順です。

1) 概要
- 使用する主要指標:
  - 傾斜角 (slope_theta)
  - 局所高さ変動 (height_variation)
  - 連続性／走行可能幅 (continuity)
  - セルの信頼度 (confidence)
  - （オプション）局所曲率 / curvature
  - （オプション）車体フットプリントによる通過性チェック

2) 合成スコア（例）
- 各指標を正規化して重み付け和で `ground_score` を計算し、閾値とヒステリシスで地面判定を行う。例式:

  normalized_slope = clamp(slope_theta / slope_thresh, 0, 1)
  normalized_var = clamp(height_variation / var_thresh, 0, 1)
  continuity_score = clamp(continuity_fraction, 0, 1)

  ground_score = 1 - (w_s * normalized_slope + w_v * normalized_var + w_c * (1 - continuity_score))

  地面条件: ground_score >= ground_score_threshold AND cell.confidence >= conf_min

  例パラメータ: slope_thresh=15°、var_thresh=0.12m、w_s=0.45、w_v=0.35、w_c=0.2、ground_score_threshold=0.6、conf_min=0.5

3) 連続性 (continuity) の算出
- 前方方向（x 正方向）に対して、指定幅（road_width_required, 例 0.6m）を満たすセルが連続している割合を算出する。
- 計算はグリッド上で中心線に対して幅方向のセル群をチェックし、該当セルの ground_score (暫定) が基準を満たす割合を求める。

4) 車体フットプリントによる通過可否チェック（推奨オプション）
- 実際の可通過性評価として、車体フットプリントをスライドさせ（lookahead）、各位置でフットプリント内のセルが十分に地面候補か（ground_ratio >= alpha）と高さ差 <= height_tol を満たすか検査する。存在すればその経路を traversable とマークし、沿道のセルを地面扱いする。

5) 速度依存閾値（動的調整）
- ロボット速度 v に応じて閾値をスケーリング:
  slope_thresh(v) = base_slope_thresh * max(0.5, 1 - k_v * v)
  height_tol(v) = base_height_tol * max(0.6, 1 - k_h * v)

6) 時間的安定化（ヒステリシス）
- 各セルについて ground_score の指数移動平均 (EMA) を保持し、high/low の閾値で地面/非地面を切り替える（反復フリップ防止）。

7) 局所曲率・形状（オプション）
- 曲率を使うと壁・垂直面の検出に有効。コストが高いので、候補セルのみ（slope 高, var 小）で計算するのがよい。

8) 疑似コード（セルベース判定）
```
for each cell (i,j):
  slope = compute_slope(i,j)
  var = compute_height_variation(i,j)
  cont = compute_continuity(i,j, required_width_cells)
  ns = clamp(slope / slope_thresh, 0,1)
  nv = clamp(var / var_thresh, 0,1)
  score = 1 - (w_s*ns + w_v*nv + w_c*(1 - cont))
  cell.ground_ema = alpha * score + (1-alpha) * cell.ground_ema_prev
  if cell.ground_ema > high_thresh and cell.confidence >= conf_min: mark ground
  elif cell.ground_ema < low_thresh: mark non-ground
```

9) 実装上の最適化案
- continuity/footprint チェックは計算量が大きくなるため、まず coarse grid（例 cell_size×2）で高速に判定し、必要領域のみ細分化して精査する。
- フットプリント評価は summed-area table（積分画像）を使うと O(1) で面積集計ができる。高さ差も同様に事前集計で高速化可能。

10) テスト・評価
- 作成するテスト: 合成シーン（縁石、段差、斜面）、実機/rosbag シーンで confusion matrix を計測
- 指標: ground detection precision/recall、false_positive_rate（地面を障害物扱い）を重要視

11) コード構造反映案
- `GridHeightMap` に `get_local_variation` と `get_continuity` を追加
- `GroundSeparator` に `evaluate_ground_cell`（ground_score 計算）と EMA/hysteresis 管理を実装
- Node は odom あるいは /cmd_vel を購読し速度に応じてパラメータを更新

まとめ: 傾斜閾値は有効だが単独では不十分。複数指標の合成、車体フットプリントの通過性評価、速度依存閾値、時間的安定化を組合せることで誤判定を大幅に低減できる。


### 6) 地面除去（点ごと）
方法:
- 点 p の属するセル (i,j) を求め、点の高さ z_p とセル高さ h_ij の差 δ = z_p - h_ij を計算。
- |δ| <= ground_max_distance かつセルが地面セルなら p を地面点としてラベル付け。

疑似コード:
```
ground_points = []
non_ground_points = []
for p in cloud:
  (i,j) = xy_to_cell(p.x, p.y)
  if cell_exists(i,j) and is_ground_cell(i,j) and abs(p.z - h[i][j]) <= ground_max_distance:
    ground_points.push(p)
  else:
    non_ground_points.push(p)
```

代表パラメータ: ground_max_distance = 0.06–0.12 m

計算量: O(M)（M は処理中点数）

### 7) クラスタリング（障害物抽出）
目的: 非地面点群から連続した物体を抽出する。

手法: Euclidean Cluster Extraction（PCL）

主要パラメータ:
- cluster_tolerance (m): 0.05–0.2
- min_cluster_size (点数): 20–100
- max_cluster_size: optional

出力: Cluster ごとに点集合・重心・バウンディングボックス（xmin,xmax,ymin,ymax,zmin,zmax）を計算。

障害物判定ルール:
- cluster_height = zmax - ground_height_at_cluster_centroid
- cluster_volume ≈ area_xy * cluster_height  (簡易)
- 障害物 if cluster_height >= min_obstacle_height AND cluster_volume >= min_obstacle_volume

代表値: min_obstacle_height = 0.08 m, min_obstacle_volume = 0.002 m^3（例）

計算量: kd-tree 構築 O(M log M)、クラスタ探索は近傍探索に依存するが一般に O(M log M)

実装注意:
- 草や枝など多数の小さなクラスタが生じやすい -> min_cluster_size と min_obstacle_volume の組でフィルタ。

### 8) 2D 投影（ScanProjector）の詳細
目的: 障害物点群を LaserScan 相当で表現し、既存のレーザーベース経路計画に接続可能にする。

手順:
- 各障害物点 p を base_link 座標系で極座標 (r, φ) に変換: r = sqrt(x^2 + y^2), φ = atan2(y, x)
- 角度範囲 [angle_min, angle_max] を分割し、角度バケット k = floor((φ - angle_min) / angle_increment) に r を投入。
- 各バケットは最小 r を保持（最小距離）。何も観測されなければ range = inf。最大距離は range_max。

疑似コード:
```
N = ceil((angle_max-angle_min)/angle_increment)
ranges = [inf]*N
for p in obstacle_points:
  r = hypot(p.x, p.y)
  phi = atan2(p.y, p.x)
  if angle_min <= phi <= angle_max:
    k = floor((phi - angle_min)/angle_increment)
    ranges[k] = min(ranges[k], r)
```

代表パラメータ: angle_min=-1.57, angle_max=1.57, angle_increment = 0.01745 (1°), range_max = 10.0

計算量: O(P)（P は障害物点数）

出力メッセージ: sensor_msgs::msg::LaserScan 形式（header.frame_id = base_link）

### 9) 境界ケースと回復戦略
- 入力点群が空: 空の出力を publish、diagnostics にログを出す。
- TF lookup 失敗: 最後に得られた変換を使う（一定時間のみ）。長時間失敗する場合はノードを警告/終了させる設定を持たせる。
- 大きな欠損領域（視界の外）: 補間で穴を埋めるが、信頼度を低く扱い threshold を厳しくする。
- 草・葉のノイズ: 小クラスタの大半を min_cluster_size でフィルタリング。必要なら点の反射強度 or color を使った二次フィルタを追加可能。

### 10) 性能最適化の実践的指針
- 入力点数に応じて leaf_size を動的に増やして処理時間を抑える（例: target_points = 30k～80k）。
- Grid のセルサイズはメモリと検出解像度のトレードオフ。小さくすると精度向上だが計算増。
- 並列化: 点群の ROI 切り出し・ダウンサンプル・地面ラベリングを独立スレッドで処理可能。
- プロファイリング指標: 各ステージ時間（ms）を diagnostics トピックで公開。目標は総処理時間 < 100 ms。

### 11) 疑似コード: パイプラインまとめ
```
on_pointcloud(cloud_msg):
  cloud = to_pcl(cloud_msg)
  cloud = transform_to_base(cloud, tf)
  cloud = crop_roi(cloud, roi)
  cloud = voxel_downsample(cloud, leaf_size)
  cloud = remove_outliers(cloud, radius, min_neighbors)
  grid = build_height_map(cloud, cell_size)
  compute_local_slope(grid)
  mark_ground_cells(grid, slope_threshold, height_var)
  (ground, non_ground) = split_points_by_grid(cloud, grid, ground_max_distance)
  clusters = euclidean_clustering(non_ground, cluster_tolerance, min_size)
  obstacles = filter_clusters_by_size_and_height(clusters, min_obstacle_height, min_volume)
  publish_pointcloud(obstacles)
  scan = project_to_scan(obstacles, angle_min, angle_max, angle_increment)
  publish_scan(scan)
  publish_diagnostics(timings)
```

### 12) パラメータサマリ（実装時に config/default_params.yaml に入れる）
- roi: x_min,x_max,y_min,y_max,z_min,z_max
- leaf_size, outlier_radius, outlier_min_neighbors
- cell_size, slope_threshold_deg, height_var_threshold, ground_max_distance
- cluster_tolerance, min_cluster_size, min_obstacle_height, min_obstacle_volume
- scan_angle_min, scan_angle_max, scan_angle_increment, scan_range_max

---


## ROS トピック・パラメータ・メッセージ

推奨トピック:
- subscribe:
  - `~/input_cloud` (sensor_msgs/PointCloud2) デフォルト: `/camera/depth/points`
- publish:
  - `~/obstacle_points` (sensor_msgs/PointCloud2) -- 障害物点群（base_link）
  - `~/obstacle_scan` (sensor_msgs/LaserScan) -- 2D 投影
  - `~/diagnostics` (diagnostic_msgs/DiagnosticArray) -- 処理時間/レート/検出数等

主要パラメータ（デフォルト値の例）:
- processing_rate: 15.0
- roi: { x_min:0.1, x_max:5.0, y_min:-1.5, y_max:1.5, z_min:-1.0, z_max:2.0 }
- voxel_leaf_size: 0.03
- outlier_radius: 0.05
- outlier_min_neighbors: 2
- grid_cell_size: 0.05
- slope_threshold_deg: 15.0
- ground_max_distance: 0.08
- cluster_tolerance: 0.1
- min_cluster_size: 30
- min_obstacle_height: 0.08
- scan_angle_min: -1.57
- scan_angle_max: 1.57
- scan_angle_increment: 0.01745

## パフォーマンス設計
- 目標: 入力/出力ともに 10Hz 以上
- 対策:
  - コア処理を C++ / PCL で実装（高速）
  - kd-tree 近傍探索、並列処理（必要に応じて）
  - 動的ダウンサンプリング（処理時間に応じた leaf_size の増減）
  - 各処理ステージの経過時間を diagnostics で公開

## テスト計画
- 単体テスト: GridHeightMap の生成・傾斜計算、GroundSeparator の判定ロジック（人工点群）
- 統合テスト: rosbag（D415 屋外データ）を用いた動作確認。出力周波数と検出精度を評価
- パフォーマンステスト: 異なる点群密度で処理時間計測（目標: 100ms 以下/フレーム）
- 評価指標: 真陽性率、偽陽性率、平均処理時間、平均周波数

## 拡張案（将来）
- 物体分類（セマンティック）フィルタ追加（深層学習）
- 動的物体の追跡と速度推定
- マルチセンサ融合（LiDAR+RGBD）

## 参考式
- 傾斜角: theta = acos(n ・ [0,0,1])
- セルインデックス: ix = floor((x - x_min) / cell_size)

---
