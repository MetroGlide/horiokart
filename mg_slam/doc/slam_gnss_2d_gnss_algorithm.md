# slam_gnss_2d GNSS拘束アルゴリズム詳細

## 概要

GNSS拘束処理は、ロボットの起動地点を基準としたSLAMのローカル座標系と、地球座標系（UTM平面直角座標）を整合させるために実行される。
従来のようなバッチ的な「2パス処理」ではなく、オンライン・オフライン問わず、スキャン入力と同時に逐次（インクリメンタル）実行されるように設計されている。

処理は主に `GnssAnchorManager` と `GnssAnchoredRunner` が担当し、以下の4つのステップで構成される。

```
[Step 1: アンカー設定]    最初の有効なGNSS fixをアンカー座標 (UTM) に決定
       ↓
[Step 2: 初期方位推定]    アンカーから init_distance_m 移動後に SLAMとGNSSの軌跡から方位 theta0 を推定
       ↓
[Step 3: 拘束の逐次追加]  新ノード追加時に、高精度GNSSデータを絶対位置拘束(PriorFactor)としてオプティマイザに逐次挿入
       ↓
[Step 4: 逐次最適化]      iSAM2等のオプティマイザで最適化。位置が一定以上変動した場合にマップを再描画
```

---

## ステップ 1 — GnssAnchorManager: アンカー設定と座標変換

**ファイル**: `gnss/anchor_manager.py` / `GnssAnchorManager`

### 1. アンカー設定
ロボット起動後、最初の有効な GNSS fix（`gpsFix` ステータスが `anchor_min_fix_status` 以上、デフォルトは 0 (NO_FIX を除く有効な測位)）を受信した時点で基準アンカーとして採用する。
アンカー決定時の緯度・経度から UTM zone を自動的に計算し、アンカーの UTM 平面座標 (`anchor_utm_easting`, `anchor_utm_northing`) を決定する。

### 2. ローカル平面座標への変換
アンカー決定以降に受信した任意の GNSS 測位（緯度・経度）を UTM 平面座標へと変換し、基準アンカーとの差分をとることで、アンカーを原点 (0, 0) とするローカル平面座標 $(x_{local}, y_{local})$ に変換する。

$$x_{local} = x_{utm} - x_{anchor\_utm}$$
$$y_{local} = y_{utm} - y_{anchor\_utm}$$

### 3. 共分散の抽出
受信した GNSS データの共分散行列（2×2）を保持する。
`navsat_fix` ソース（`sensor_msgs/NavSatFix`）の場合は、メッセージの ENU 順 3×3 共分散行列から 2D 処理に必要な East/North の 2×2 ブロックを取り出す。
`navpvt` ソース（`ublox_msgs/NavPVT`）の場合は、水平精度 `h_acc` [mm] から $std\_dev = h\_acc \times navpvt\_hacc\_scale \times 10^{-3}$ を計算し、等方性の 2×2 共分散行列を生成する。

---

## ステップ 2 — GnssAnchoredRunner: 初期方位の推定とグラフ初期化

**ファイル**: `gnss/gnss_anchored_runner.py` / `GnssAnchoredRunner`

起動直後は SLAM 座標系と GNSS 座標系の間の回転オフセット（絶対方位）が不明であるため、システムは `INITIALIZING` 状態となる。

### 1. 初期方位の推定
ロボットが基準アンカーから `init_distance_m` (デフォルト: 2.0 m) 以上移動するのを待つ。
移動が完了した時点で、最初のノードから現在ノードまでの SLAM 座標系における変位角 $\theta_{slam}$ と、GNSS ローカル平面座標における変位角 $\theta_{gnss}$ を比較し、回転オフセット `init_rotation` を推定する。

$$\theta_{gnss} = \text{atan2}(y_{local}, x_{local})$$
$$\theta_{slam} = \text{atan2}(y_{slam} - y_{slam,0}, x_{slam} - x_{slam,0})$$
$$\theta_0 = \theta_{gnss} \quad (\text{初期ノードに設定する絶対方位})$$
$$\text{init\_rotation} = \theta_0 - \text{node}_0.yaw$$

### 2. ポーズグラフの初期化
アライナーとしての回転角 `init_rotation` が確定すると、それまでに蓄積されていた全ての SLAM ノードの姿勢を回転・変換し、アンカー基準の絶対方位にアライメントする。
オプティマイザの初期設定として、最初のノード位置 (0, 0, $\theta_0$) に強い事前拘束（PriorFactorPose2, $\sigma_{pos} = \text{anchor\_sigma\_m}$, $\sigma_{yaw} = \text{init\_yaw\_sigma\_rad}$）を加えて初期化する。

---

## ステップ 3 — GnssAnchoredRunner: インクリメンタルな拘束追加

グラフが `RUNNING` 状態へ移行した後は、キーフレーム（ノード）が追加されるたびに以下の手順で Prior 拘束を追加する。

### 1. 高精度測位データの判定とフィルタリング
対応するスキャン時刻に最も近いタイムスタンプを持つ GNSS 測位データを検索する。
GNSS データの共分散 $C_{xx}$ が有効（正の値）であれば、標準偏差 $\sigma_{xy} = \sqrt{C_{xx}}$ とする。
共分散がゼロ（無効）の場合は、`fix_status` に応じてフォールバック用精度を設定する。
*   `fix_status >= 2` (RTK-Fixed) : $\sigma_{xy} = \text{gnss\_fix\_sigma\_m}$ (デフォルト: 0.02 m)
*   `fix_status >= 0` (RTK-Float/Single) : $\sigma_{xy} = \text{gnss\_float\_sigma\_m}$ (デフォルト: 0.5 m)

標準偏差 $\sigma_{xy}$ が `gnss_max_sigma_m` (デフォルト: 2.0 m) を超えるデータ、あるいはステータスが無効なデータは、ポーズグラフを歪める原因になるため拘束の追加をスキップする。

### 2. Prior 拘束の追加
フィルタを通過した GNSS 位置データ $(x_{local}, y_{local})$ を、該当するノードインデックスに対して絶対位置拘束として追加する。
GNSS ではロボットの方位（yaw）を直接観測できないため、オプティマイザに設定する 3×3 情報行列（GTSAMのPriorFactorPose2用）の yaw 分散を極めて大きな値（`gnss_factor_yaw_variance`、デフォルト: $10^8 \text{ rad}^2$）に設定し、位置（x, y）のみを拘束する。

---

## ステップ 4 — IncrementalOptimizer: 逐次最適化とマップ再描画

**ファイル**: `optimizer/isam2_optimizer.py` または `optimizer/gtsam_incremental_adapter.py`

オプティマイザは、各ノードの追加やループ辺・GNSS拘束の追加が行われるたびに、`update()` を介してファクターグラフ全体の非線形最適化を実行する。

### 1. 最適化によるポーズの更新
オプティマイザから得られた最適化後のポーズ $(x, y, \theta)$ を `PoseNode` に反映する。

### 2. 変動量の判定と再描画の抑制
最適化の結果、ロボット位置が前回マップをレンダリングした位置から `gnss_rerender_threshold_m` (デフォルト: 0.1 m) 以上変化した場合にのみ、マップレンダラーの `rerender_all()` をトリガーして占有格子マップ全体の再描画を要求する。これにより、毎スキャンごとの不要なレンダリング計算コストを削減し、CPU負荷を最小限に抑える。

---

## 出力アーティファクトとナビゲーション連携

### 1. SlamDataSaver によるアーティファクト保存
オフライン bag 再処理の完了時、またはオンライン SLAM のマップ保存サービス（`/slam_gnss_2d/save_slam_map`）がコールされた際、`SlamDataSaver` は以下の2つのファイルをマップファイルと共に出力する。
*   **`gnss_transform.yaml`**: マップ座標系と地球座標系 (UTM/WGS84) の座標変換パラメータ。
    *   `anchor` : 基準アンカーの緯度、経度。
    *   `anchor_utm` : 基準アンカーの UTM 平面座標 (easting, northing, zone, hemisphere)。
    *   `rotation_rad` : 初期回転角（SLAMローカル軌跡とGNSS軌跡の回転差）。
*   **`pose_graph.json`**: ポーズグラフの履歴データ（ノードの座標、タイムスタンプ、ノード間エッジ）。

### 2. SlamGnssNavBridgeNode による自律移動時の連携
自律移動（Navigation2）実行フェーズでは、`slam_gnss_nav_bridge_node` を起動する。本ノードは上記の `gnss_transform.yaml` から座標変換パラメータを読み込み、動作を開始する。
ロボットが受信する生の GNSS 座標（`/gps/fix` または `/navpvt`）を、アンカー位置と回転角を基にマップ座標系のオドメトリ情報に変換する。
変換されたオドメトリは、`/odom/gps` トピック (`nav_msgs/Odometry`) として配信され、`robot_localization` などの外部カルマンフィルタに入力される。これにより、マップと地球座標系が一致した状態での高精度な自律移動が可能になる。

---

## 精度に影響する要因

| 要因 | 影響 | 対処 |
| --- | --- | --- |
| 初期方位推定時の直線移動不足 | `INITIALIZING` 状態から遷移せず、グラフが固定されない | アンカー設定後、ロボットを直線的に数メートル（`init_distance_m` 以上）移動させる。 |
| GNSS 測位のマルチパスや劣化 | 一時的に位置がジャンプし、グラフが歪む原因になる | `gnss_max_sigma_m` を超えた低精度データを自動的に除外。RTK ステータスを信頼する。 |
| GNSS の一時的な遮断 (ロスト) | 拘束が追加されず、スキャンマッチング/オドメトリのみになる | 遮断が `missing_grace_frames` 続いた場合は、自動的にデグレード（スキャン/オドメトリのみ）モードに移行し、復帰時に自動復帰する。 |
