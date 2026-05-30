# slam_gnss_2d GNSS拘束アルゴリズム詳細

## 概要

GNSS拘束処理は rosbag 再生によるポーズグラフ構築が完了した後、
**2パス目**として一括実行される。ローカル座標系だけで構築されたSLAMマップを
地球座標系（UTM平面直角座標）に整合させることが目的。

処理は4つのステップに分かれ、それぞれ独立したクラスが担当する。

```
BagGnssSource           → GnssData[]          (lat/lon → UTM変換済み)
       ↓
KinematicHeadingAligner → (tx, ty, rotation_rad)  (座標系変換パラメータ推定)
       ↓
GnssConstraintInserter  → GnssPrior[]          (SLAM座標系に変換された拘束リスト)
       ↓
GTSAMOptimizer          → PoseNode[]           (GNSS拘束付き再最適化済みノード)
```

---

## ステップ 1 — BagGnssSource: lat/lon を UTM 座標に変換する

**ファイル**: `input/ros2/bag_reader.py` / `BagGnssSource`

### 処理内容

rosbag2 から `sensor_msgs/NavSatFix` メッセージを全件読み込み、
球面座標（緯度・経度）をデカルト平面座標（UTM）に変換して内部バッファに格納する。
`start()` 呼び出し時に一括処理し、以後は `get_all_gnss()` で取り出す。

### アルゴリズム

**1. STATUS_NO_FIX のフィルタリング**

`msg.status.status < 0`（`STATUS_NO_FIX = -1`）のメッセージを除外する。
精度が不定の fix をグラフ拘束に使わないための前処理。

**2. UTM zone の自動決定**

最初の有効な fix の経度から zone 番号を計算する。

```
zone = floor((longitude + 180) / 6) + 1
south = (latitude < 0)
```

南緯の場合は `south=True` を設定する。事前に zone を指定する設定項目はなく、
bag 内のデータから自動決定する。

**3. pyproj による座標変換**

`Transformer.from_crs('EPSG:4326', crs_utm, always_xy=True)` を構築し、
`(longitude, latitude)` → `(easting, northing)` = `(x, y)` に変換する。
UTM の原点はzone西端の赤道交点（北半球）であり、近距離変換の誤差が小さい。

**4. 共分散行列の取り出し**

`NavSatFix.position_covariance` は ENU（East-North-Up）順の 9 要素フラット配列。
2D処理に必要な East/North の 2×2 ブロックを取り出す。

```
position_covariance のインデックス配置（ENU順 3x3）:

    E    N    U
E [0]  [1]  [2]
N [3]  [4]  [5]
U [6]  [7]  [8]

取り出す 2x2:
C_GNSS = [[cov[0], cov[1]],   (σ_E², σ_EN)
          [cov[3], cov[4]]]   (σ_NE, σ_N²)
```

**5. タイムスタンプ昇順ソート**

bag 内の順序は保証されないため、取り込み後にソートする。
以後の最近傍検索は昇順ソート済みを前提とする。

### 出力

`get_all_gnss()` → `list[GnssData]`（UTM x/y, 共分散行列 2×2, タイムスタンプ）

---

## ステップ 2 — KinematicHeadingAligner: 座標系変換を推定する

**ファイル**: `gnss/kinematic_aligner.py` / `KinematicHeadingAligner`

### 問題設定

GNSSはUTM座標系、SLAMはロボット起動地点を原点とした任意向きのローカル座標系で動作する。
この2座標系の間には「回転 + 平行移動」の剛体変換が存在し、
ロボットが実際に移動した経路を両座標系で比較することで推定できる。

変換の定義:

```
SLAM_xy = R(θ) * GNSS_xy + (tx, ty)

R(θ) = [[cos θ, -sin θ],
        [sin θ,  cos θ]]
```

### アルゴリズム（2段階）

#### 第1段階: 回転角 θ の推定

連続する GNSS 測位ペア `(g0, g1)` と、
それぞれに時刻が最も近いSLAMノードのペア `(n0, n1)` を使う。

**速度フィルタ**

ロボットが実際に移動した区間のみ採用する。
低速時はGNSSノイズが方位推定を悪化させるため除外する。

```
speed = sqrt(Δgx² + Δgy²) / Δt  ≥  v_min
```

デフォルト: `v_min = 0.5 m/s`（パラメータ `kinematic_min_speed_ms`）

**方位差の収集**

各ペアから GNSS 方位とSLAM方位を計算し、その差を回転サンプルとして収集する。

```
φ_GNSS = atan2(Δgy, Δgx)        # GNSS座標系での移動方位
φ_SLAM = atan2(Δsy, Δsx)        # SLAM座標系での移動方位
θ_i    = φ_SLAM - φ_GNSS        # 1ペアの回転サンプル
```

SLAM_xy = R(θ) * GNSS_xy + t なので、同一方向の移動に対して
`φ_SLAM = φ_GNSS + θ` が成り立つ。

**円形平均（circular mean）**

収集した全サンプルの平均を求める。単純平均では ±π 付近でラップアラウンド誤差が
生じるため、単位ベクトルの和から角度を求める円形統計を使う。

```
θ = atan2( Σ sin(θ_i),  Σ cos(θ_i) )
```

有効サンプルが0件の場合は `(0, 0, 0)` を返す（変換なしとして扱う）。

#### 第2段階: 平行移動 (tx, ty) の推定

回転角 θ が定まったので、全 GNSS 点を回転させた後の残差から平行移動量を推定する。
各 GNSS 測位 `g_k` と最近傍ノード `n_k` について:

```
t_k = (n_k.x, n_k.y) - R(θ) * (g_k.x, g_k.y)

    = (n_k.x - (cos(θ)*g_k.x - sin(θ)*g_k.y),
       n_k.y - (sin(θ)*g_k.x + cos(θ)*g_k.y))
```

全点の平均:

```
(tx, ty) = (1/N) * Σ t_k
```

### 最近傍ノード検索

タイムスタンプリストに対して `bisect_left` で O(log N) 検索を行い、
前後のノードのうち時刻差が小さい方を選ぶ。SLAMのノード追加周期がGNSSより
疎な場合でも同じ実装で対応できる。

### 出力

`(tx, ty, rotation_rad)` — ステップ3に渡す変換パラメータ

---

## ステップ 3 — GnssConstraintInserter: GNSS拘束を生成する

**ファイル**: `gnss/constraint_inserter.py` / `GnssConstraintInserter`

### 処理内容

ステップ2で得た変換を使って全 GNSS 測位をSLAM座標系に変換し、
各 GNSS 点に対応するSLAMノードへの `GnssPrior` を生成する。

**設計上の重要な制約**: このクラスはGTSAMへの依存を持たない。
ポーズグラフへの直接操作はせず、optimizer 層への入力データを生成するだけ。
これにより gnss/ 層と optimizer/ 層の独立性を保つ。

### アルゴリズム

**GNSS座標の変換**

ステップ2で得た `(tx, ty, rotation_rad)` を適用する。

```
x_slam = cos(θ) * gx  -  sin(θ) * gy  +  tx
y_slam = sin(θ) * gx  +  cos(θ) * gy  +  ty
```

**共分散行列の座標変換**

GNSS共分散 `C_GNSS`（ENU系・2×2）をSLAM座標系に回転変換する。

```
C_slam = R * C_GNSS * R^T
```

ENU系とSLAM系の軸方向が異なるため、この変換は必須。
省略すると情報行列の主軸方向が誤った向きに設定される。

**情報行列の算出**

共分散の逆行列として情報行列を求める。
行列式が極端に小さい（数値的に不安定）場合はデフォルト精度を使用する。

```
I = C_slam^{-1}         if det(C_slam) >= 1e-9
  = (1/σ₀²) * E         otherwise

σ₀ = gnss_noise_xy_m（デフォルト 3.0 m）
```

**ノード対応付け**

ステップ2と同じ bisect ベースの最近傍検索で、各 GNSS 測位を最も時刻が近い
`PoseNode` に対応付ける。複数の GNSS 測位が同一ノードに対応することもある。

### 出力

`list[GnssPrior]` — ノードインデックス、SLAM系x/y座標、情報行列 2×2 のリスト

---

## ステップ 4 — GTSAMOptimizer: GNSS拘束付き再最適化

**ファイル**: `optimizer/gtsam_optimizer.py` / `GTSAMOptimizer`

### 処理内容

ポーズグラフの全ノード・辺と `GnssPrior` リストを受け取り、
LevenbergMarquardt 法でグラフ最適化を実行する。
`gnss_priors` が空のときは従来のループクローズ最適化と同一動作になる。

### ファクターグラフの構成

```
ファクター種別                 対象                  重み
──────────────────────────────────────────────────────────────
PriorFactorPose2 (アンカー)   nodes[0]              分散 [1e-6, 1e-6, 1e-8]
BetweenFactorPose2             全 PoseEdge           edge.information (3×3)
PriorFactorPose2 (GNSS)        GnssPrior対応ノード   下記参照
```

**アンカー拘束**: 最初のノードを強く固定してゲージ自由度を除去する。
これがないと最適化の解が並進・回転方向に一意に定まらない。

**GNSS拘束（PriorFactorPose2）**: 各ノードに絶対位置の弱い拘束を加える。
GNSS精度に応じた重みで、ループクローズ辺と競合しながら最適な姿勢を決定する。

### GNSS拘束のノイズモデル

`GnssPrior.information` は xy の 2×2 情報行列だが、
`PriorFactorPose2` は 3×3（x, y, yaw）の情報行列を要求する。
yaw は GNSS では観測できないため、yaw の分散を非常に大きく設定して実質的に自由にする。

```
I_3x3 = [[I_2x2,      0    ],
          [0,      1/σ_θ²  ]]

σ_θ² = 1e6 rad²   (_GNSS_YAW_VARIANCE)
```

この設定により、x/y はGNSS精度で拘束されるが、yaw はスキャンマッチングや
オドメトリの辺だけで決まる。

### 初期値の設定順序（重要）

```python
# STEP 1: 全ノードの初期値を先に登録する
for node in nodes:
    initial.insert(node.index, Pose2(node.x, node.y, node.yaw))

# STEP 2: アンカー・辺・GNSS拘束を追加する
# GNSS拘束で initial.atPose2(idx) を呼ぶため、STEP 1 より後でなければならない
for gnss_prior in gnss_priors:
    node_initial = initial.atPose2(gnss_prior.node_index)  # yaw は初期値から引用
    graph.add(PriorFactorPose2(
        gnss_prior.node_index,
        Pose2(gnss_prior.x, gnss_prior.y, node_initial.theta()),  # yaw は拘束しない
        gnss_noise,
    ))
```

GNSS Prior の Pose2 には初期値の yaw をそのまま設定することで、
yaw 方向の Factor の誤差を 0 にして最適化に影響しないようにする。

### 最適化

LevenbergMarquardt 法（GTSAMデフォルトパラメータ）で収束まで反復する。
verbosity を `SILENT` に設定しログ出力を抑制する。

### 出力

`list[PoseNode]` — 最適化後の全ノード（インデックス・タイムスタンプは入力と同一）

---

## GNSS 2パス処理フロー（`SlamOfflineNode._run_gnss_phase`）

```
bag 再生完了（_process_step が False を返す）
        │
        ▼
use_gnss == True ?
        │ Yes
        ▼
gnss_list = gnss_source.get_all_gnss()
        │ 空なら warn ログを出して終了
        ▼
nodes = pose_graph.get_nodes()
edges = pose_graph.get_edges()
        │ nodes が空なら warn ログを出して終了
        ▼
transform = gnss_aligner.estimate_transform(nodes, gnss_list)
        │ → ログ: "GNSS align: tx=X.XXm ty=X.XXm rot=X.XXdeg (N fixes, M nodes)"
        ▼
priors = gnss_inserter.build_priors(nodes, gnss_list, transform)
        │ → ログ: "GNSS inserting N prior constraints"
        ▼
updated = gnss_optimizer.optimize(nodes, edges, gnss_priors=priors)
        │
        ├─ renderer.rerender_all(updated)   ← 全ノードのスキャンでマップを再描画
        ├─ _rebuild_path(updated)           ← nav_msgs/Path を再生成
        └─ map_dirty = True                 ← 次のパブリッシュで /map を配信
        │
        ▼
ログ: "GNSS phase complete: map re-rendered with GNSS constraints"
```

---

## パラメータ一覧

パラメータは `slam_gnss_2d_offline_node` の `ros__parameters` に記述する
（[params/slam_gnss_2d.yaml](../params/slam_gnss_2d.yaml) 参照）。

| パラメータ               | デフォルト | 意味                                           |
| ------------------------ | ---------- | ---------------------------------------------- |
| `use_gnss`               | `false`    | GNSS 2パス処理を有効にする                     |
| `gnss_topic`             | `/gps/fix` | NavSatFix トピック名                           |
| `gnss_noise_xy_m`        | `3.0`      | 共分散が無効な場合のフォールバック測位精度 [m] |
| `kinematic_min_speed_ms` | `0.5`      | 回転推定サンプルに使う最低移動速度 [m/s]       |

---

## 精度に影響する要因

| 要因                                | 影響                                        | 対処                                            |
| ----------------------------------- | ------------------------------------------- | ----------------------------------------------- |
| GNSS精度（open sky vs. マルチパス） | `GnssPrior.information` の大きさが変わる    | NavSatFix の covariance を信頼する（補正不要）  |
| ロボットの直線移動量が少ない        | 回転推定サンプルが集まらず `(0,0,0)` を返す | `kinematic_min_speed_ms` を下げる               |
| ポーズグラフノード数が少ない        | GNSS測位との時刻対応が粗くなる              | `LoopClosureBuilder` のノード追加間隔を調整する |
| UTM zone 境界付近での走行           | 変換誤差が大きくなる                        | 現状未対処（実用上まれ）                        |
