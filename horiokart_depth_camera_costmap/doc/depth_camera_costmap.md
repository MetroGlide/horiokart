# 深度カメラによる地形・障害物検出システム仕様書

## 1. 概要
本システムは、屋外の不整地を移動するロボットの自律走行を支援するため、Realsense D435カメラの点群データと2DLIDARデータを活用し、観測された地形を「走行可能な路面」と「障害物」に分類します。base_link座標系を唯一の安定した基準とした相対評価により、ロボットの現在の姿勢に合わせた走行可否判定を行います。
本ノードは2D自己位置推定（AMCL等）およびNavigation2によるナビゲーションシステムと統合され、コストマップ情報をナビゲーションスタックへ提供します。

## 2. 前提条件
- ロボットの走行能力（最大傾斜角・最大段差高さ）が既知であること
- base_linkのZ軸は重力方向と必ずしも一致しない
- 2D自己位置推定（AMCL等）およびNavigation2によるナビゲーションシステムが稼働していること
- ロボットの座標系（base_link）と地図座標系（map）のTFが正しく管理されていること
- 深度カメラと2DLIDARのキャリブレーションが行われていること

## 3. 入出力
### 3.1 入力データ
- Realsense D435カメラデータ（sensor_msgs/msg/PointCloud2形式、一定周期で取得）
- 2DLIDARデータ（sensor_msgs/msg/LaserScan形式、一定周期で取得）
- ロボットの姿勢情報（TFによるbase_linkからカメラまでの位置・姿勢）
- 地図座標系（map）とロボット座標系（base_link）のTF（2Dナビゲーション統合用）

### 3.2 出力データ
- コストマップ（nav_msgs/msg/OccupancyGrid形式の2次元グリッドマップ、Navigation2/costmap_2dへ提供）
- オブジェクト情報（visualization_msgs/msg/Marker形式などで表現される障害物の位置・形状・分類、RViz等で可視化）

## 4. アルゴリズム概要
### 4.1 基本方針
- 特定の「地面」を前提とせず、観測された地形すべてを走行可能性で評価
- base_link座標系を唯一の基準とし、すべての判断をその基準に対する相対的な関係で行う
- 2Dナビゲーションシステムのコストマップレイヤーとして動作し、他レイヤー（静的地図・障害物等）と合成される
- 深度カメラ層と2DLIDAR層の情報を安全冗長的に合成

## 5. 処理ステップ
### 5.1 点群の前処理とグリッドマップ生成
1. ROS 2トピックから色付き点群を受信
2. pcl::VoxelGridフィルタで点群をダウンサンプリング
3. pcl::StatisticalOutlierRemovalフィルタでノイズ除去
4. tf2_rosでcamera_link→base_link座標系への変換を取得
5. pcl::transformPointCloudで点群をbase_link座標系に変換
6. base_linkのXY平面に投影し、一定解像度（例: 0.1m/セル）で2次元グリッドマップを初期化
7. 各グリッドセルに属する点群から以下の特徴量を抽出
   - Z_min, Z_max（高さ）
   - Z_variance（高さ分散）
   - Mean_Normal（法線ベクトル、pcl::NormalEstimation使用）
   - Mean_RGB（平均色）

### 5.2 地形の識別と相対評価
1. 走行可能な路面の認識
   - Z_varianceが低く（例: < 0.005）、Mean_Normalベクトルの傾きが最大傾斜角以内のセルを走行可能と分類
2. 変曲点と障害物の認識
   - 隣接セルのMean_Normalベクトルの急変（角度差が閾値以上）を変曲点として識別
   - 変曲点の前後でZ_min差が最大段差高さ超過→段差障害物
   - 変曲点以降の傾斜が最大傾斜角超過→急傾斜障害物
   - 急激なZ_min変化、Z_maxが重心より高い領域→障害物

### 5.3 トラバーサビリティ評価とコストマップ生成
1. スロープ：相対傾斜角と最大傾斜角で評価
2. 段差：相対高さと最大段差高さで評価
3. コスト割当
   - 走行可能な路面：低コスト（例: 0-50）
   - 乗り越え可能なスロープ/段差：中コスト（例: 51-127）
   - 乗り越え不能な障害物：高コスト（例: 128-254）、通行不可（255）
4. オブジェクト情報：障害物領域をクラスタリングし、位置・形状・分類をvisualization_msgs/msg/Marker等で出力

## 6. 参考
- 使用ライブラリ：PCL, ROS2 TF, sensor_msgs, visualization_msgs, nav_msgs, costmap_2d, tf2_ros
- カメラ設置：ロボット上部から正面斜め下向き
- 2D自己位置推定（AMCL等）・Navigation2との連携を前提

---
### 備考（コストマップ合成・安全設計の補足）

#### レイヤー合成順序と周期
- costmap_2dのレイヤー合成は「pluginsリストの順番」でupdateされ、後からsetCostした値が有効となる。
- 周期の違い（例：LIDAR層10Hz、深度カメラ層1Hz）は「最新データの反映頻度」に影響するが、1回のコストマップ更新サイクルでは合成順序が優先される。
- LIDAR層→深度カメラ層の順で合成し、深度カメラ層で条件付き上書きを行う。

#### 斜面領域の低コスト上書き
- 深度カメラ層で斜面領域を低コストでsetCostすれば、LIDAR層の誤検知（高コスト）は上書きされる。
- 深度カメラ層の更新周期が遅い場合でも、前回の判定値が維持され、LIDAR層の高コストで上書きされることはない。

#### 障害物の即時反映リスク
- 深度カメラ層の更新後に新たな障害物が現れ、LIDAR層で検出されても、深度カメラ層の次回更新までコストマップに反映されないリスクがある。
- このリスクを低減するには、深度カメラ層の更新周期を速くする、または障害物領域はLIDAR層のコストを優先する設計が有効。

#### 条件付き上書きの設計
- updateCosts内で「深度カメラ点群が存在するセルのみsetCostで上書き」「未検出・未更新セルはLIDAR層のコストを維持」するロジックとする。
- これにより、斜面領域は誤検知を防ぎつつ、障害物の即時反映・安全性も確保できる。
- 主な運用例：
  - 斜面領域：深度カメラで走行可能と判定→低コストで上書き
  - 障害物領域：LIDAR/深度カメラ両方で障害物→高コスト
  - 深度カメラ未検出領域：LIDARのみ検出→LIDARコスト維持

#### パラメータ設計補足
- LIDAR/Depth層の合成順序・優先度はyaml/launchで明示的に設定
- 条件付き上書きの有効/無効をパラメータ化して運用可能

#### 運用方針
- 深度カメラと2DLIDARのキャリブレーションを前提とし、両センサの冗長性・安全性を活かす設計とする。
- パラメータや合成順序はyaml/launchで柔軟に調整可能。

---
## 7. 主要パラメータ一覧

| パラメータ名           | 説明                                 | 例・初期値         |
|------------------------|--------------------------------------|--------------------|
| 最大傾斜角             | ロボットが走行可能な最大斜面角度     | 20度               |
| 最大段差高さ           | 乗り越え可能な段差の最大高さ         | 0.10m              |
| グリッド解像度         | コストマップのセルサイズ             | 0.10m/セル         |
| Z_variance閾値         | 路面判定用の高さ分散閾値             | 0.005              |
| 法線ベクトル角度閾値   | 変曲点判定用の隣接セル法線角度差     | 10度               |
| コスト値（路面/障害物）| コストマップの割当値                 | 0-255              |
| 点群ダウンサンプリング | VoxelGridフィルタのリーフサイズ      | 0.05m              |
| 外れ値除去閾値         | StatisticalOutlierRemovalの閾値      | 1.0                |

※値はロボット仕様や実験により調整してください。
---
## 8. Navigation2 costmap_2dプラグインとしての実装方法補足

本ノードは、Navigation2のcostmap_2dプラグインとして実装することで、標準ナビゲーションスタック（2D自己位置推定・経路計画等）と連携しやすくなります。

### 実装のポイント
- costmap_2d::Layerクラスを継承し、独自の点群処理・コストマップ生成ロジックを`updateBounds`・`updateCosts`で実装します。
- ROS2のpluginlibを用いてプラグイン登録（`PLUGINLIB_EXPORT_CLASS`マクロ）を行います。
- 点群受信はROS2のサブスクライバ（sensor_msgs/msg/PointCloud2）で行い、必要に応じてTF変換（tf2_ros）を利用します。
- 生成した2次元コストマップは、costmap_2dのレイヤーとして他レイヤー（障害物・静的地図等）と合成され、2Dナビゲーションシステムの経路計画・障害物回避に利用されます。
- パラメータ（閾値・解像度等）は、costmap_2dのyaml設定やlaunchファイルから動的に取得できるようにします。

### 参考実装例
1. `include/your_package/depth_camera_layer.hpp`でLayer継承クラスを定義
2. `src/depth_camera_layer.cpp`で点群処理・コストマップ生成・レイヤー更新処理を実装
3. `plugin.xml`でプラグイン登録
4. `nav2_costmap_2d`の`plugins`パラメータに本プラグインを追加

詳細はNavigation2公式ドキュメント（https://navigation.ros.org/）の「Costmap2D Plugin」セクションを参照してください。

---
## 9. 詳細設計（ノード実装）

### 9.1 構成概要
本ノードは、ROS2 Humble環境・C++で実装し、Navigation2のcostmap_2dプラグインとして動作する。2D自己位置推定・ナビゲーションシステムと統合され、コストマップ情報をナビゲーションスタックへ提供する。主な構成要素は以下：
- 点群受信・前処理
- TF変換による座標系統一
- グリッドマップ生成・特徴量抽出
- 地形・障害物判定
- コストマップ生成・出力（Navigation2/costmap_2dレイヤーとして）
- 障害物クラスタリング・可視化情報出力

### 9.2 ノード・クラス設計
- `DepthCameraLayer`（costmap_2d::Layer継承）
  - 主処理クラス。updateBounds/updateCostsでコストマップ更新。
- `PointCloudProcessor`
  - 点群のダウンサンプリング・ノイズ除去・座標変換・特徴量抽出を担当。
- `TraversabilityEvaluator`
  - 各セルの走行可否判定・コスト割当。
- `ObstacleClusterer`
  - 障害物領域のクラスタリング・visualization_msgs/msg/Marker生成。
- `ParameterManager`
  - パラメータ管理（閾値・解像度等の取得・更新）。

### 9.3 トピック・インターフェース設計
- サブスクライブ：
  - `/camera/depth/color/points`（sensor_msgs/msg/PointCloud2）
  - TF（camera_link→base_link, map→base_link）
- パブリッシュ：
  - コストマップ（costmap_2d内部で管理、Navigation2へ提供）
  - 障害物情報（`/detected_obstacles`、visualization_msgs/msg/MarkerArray、RViz等で可視化）
- サービス/アクション：
  - パラメータ動的更新（optional, rclcpp::ParameterEvent）

### 9.4 パラメータ設計

| パラメータ名（変数名）                | 型        | 概要・用途                                            | 例・初期値         |
|--------------------------------------|-----------|------------------------------------------------------|--------------------|
| max_slope_angle_deg                  | double    | ロボットが走行可能な最大斜面角度（度）                | 20.0               |
| max_step_height_m                    | double    | 乗り越え可能な段差の最大高さ（m）                     | 0.10               |
| grid_resolution_m                    | double    | コストマップのセルサイズ（m/セル）                    | 0.10               |
| z_variance_threshold                 | double    | 路面判定用の高さ分散閾値                             | 0.005              |
| normal_angle_threshold_deg           | double    | 変曲点判定用の隣接セル法線角度差（度）                | 10.0               |
| cost_traversable                     | int       | 走行可能セルのコスト値                                | 0-50               |
| cost_semi_traversable                | int       | 乗り越え可能セルのコスト値                            | 51-127             |
| cost_obstacle                        | int       | 乗り越え不能セルのコスト値                            | 128-254            |
| cost_lethal                          | int       | 通行不可セルのコスト値                                | 255                |
| voxel_leaf_size_m                    | double    | VoxelGridフィルタのリーフサイズ（m）                  | 0.05               |
| sor_mean_k                           | int       | StatisticalOutlierRemovalの近傍点数                   | 50                 |
| sor_stddev_mul_thresh                | double    | StatisticalOutlierRemovalの閾値                       | 1.0                |
| cluster_distance_threshold_m          | double    | 障害物クラスタリングの距離閾値（m）                   | 0.20               |
| cluster_min_points                   | int       | 障害物クラスタリングの最小点数                        | 10                 |

パラメータはyaml/launchから取得し、rclcpp::Node::declare_parameter/ get_parameterで管理する。

### 9.5 点群処理フロー詳細
1. sensor_msgs/msg/PointCloud2受信（callback）
2. pcl::VoxelGridでダウンサンプリング
3. pcl::StatisticalOutlierRemovalでノイズ除去
4. tf2_ros::Buffer/Listenerでcamera_link→base_link変換取得
5. pcl::transformPointCloudで座標変換
6. XY平面に投影し、グリッドマップ初期化
7. 各セルごとに点群抽出、特徴量計算：
   - Z_min, Z_max, Z_variance
   - Mean_Normal（pcl::NormalEstimation）
   - Mean_RGB

### 9.6 コストマップ生成・更新処理
1. 各セルの特徴量から走行可否判定：
   - Z_variance < 閾値、Mean_Normal傾き < 最大傾斜角 → 走行可能
   - 隣接セルのMean_Normal角度差 > 閾値 → 変曲点
   - 変曲点前後のZ_min差 > 最大段差高さ → 段差障害物
   - 変曲点以降の傾斜 > 最大傾斜角 → 急傾斜障害物
   - Z_min急変、Z_max高い → 障害物
2. コスト割当：
   - 走行可能：低コスト（0-50）
   - 乗り越え可能：中コスト（51-127）
   - 乗り越え不能：高コスト（128-254）、通行不可（255）
3. costmap_2d::Costmap2DのsetCostで反映（条件付き上書き：深度カメラ点群が存在するセルのみ上書き、未検出セルはLIDARコスト維持）

### 9.7 障害物クラスタリング・オブジェクト情報出力
1. 高コストセル領域をクラスタリング（DBSCAN等）
2. クラスタごとに位置・形状・分類（急斜面/岩/壁等）を推定
3. visualization_msgs/msg/MarkerArrayで障害物情報をpublish

### 9.8 エラー処理・例外設計
- 点群未受信・TF未取得時は処理スキップ/警告ログ
- pcl/TF変換失敗時は例外キャッチ・エラーログ
- パラメータ不正値は初期値でフォールバック
- コストマップ更新失敗時は警告のみ（他レイヤーに影響しない設計）

### 9.9 テスト・デバッグ方針
- 単体テスト：各クラスごとにgtestでロジック検証
- 統合テスト：実機/シミュレータで点群→コストマップ変換確認
- RVizでコストマップ・障害物Marker可視化
- ログ出力（INFO/WARN/ERROR）で処理状況確認
- パラメータ変更による挙動確認（launch/yaml切替）
- 2D自己位置推定・ナビゲーションシステムとの統合テスト（経路計画・障害物回避挙動の確認）
- 障害物即時反映の検証（LIDARのみ検出時の安全性確認）

