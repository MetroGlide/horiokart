# PoseGraphDiff.msg
# 新規追加されたノード情報（毎キーフレーム）
int32[] new_node_indices
float64[] new_node_x
float64[] new_node_y
float64[] new_node_yaw
float64[] new_node_timestamps

# 新規追加されたシーケンシャルエッジ（毎キーフレーム）
int32[] seq_edge_from
int32[] seq_edge_to
float64[] seq_edge_score       # ICPスコア（小さいほど良い, 0.0=未計算）
int32[] seq_edge_type          # 0=ICP_SUCCESS, 1=ICP_ODOM_FALLBACK
float64[] seq_edge_info_diag   # 情報行列の対角3要素 [info_x, info_y, info_yaw] のフラット配列 (要素数は 3 * edge数)

# 新規追加されたGNSS Prior（GNSSあり時のみ）
int32[] prior_node_indices
float64[] prior_sigma_m        # 測位精度σ [m]
int32[] prior_gnss_status      # NavSatFix.status互換 (-1=未設定, 0=fix, 1=float)

# 新規追加されたループエッジ（ループ発生時のみ）
int32[] loop_edge_from
int32[] loop_edge_to
float64[] loop_edge_score
float64[] loop_edge_info_diag  # 情報行列対角3要素のフラット配列

# イベントフラグ
bool loop_closed               # このフレームでループクロージャが発生したか
bool full_refresh_needed       # Trueの場合、UIは保持中のデータを破棄してフル取得すべき
