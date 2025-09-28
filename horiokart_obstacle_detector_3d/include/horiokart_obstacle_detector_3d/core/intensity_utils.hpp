#pragma once

#include <Eigen/Dense>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector {
// 純粋関数: 正規化された強度 [0,1] に対して距離と角度による補正を適用します。
// パラメータ:
// - intensity: 入力強度 [0,1]
// - point: 点の座標（ターゲットフレーム）
// - sensor_origin: センサ座標（ターゲットフレーム）
// - sensor_forward: センサの前方単位ベクトル（ターゲットフレーム）
// - distance_ref: 距離補正の参照距離
// - distance_power: 距離補正の指数
// - compensate_distance: 距離補正の有無
// - compensate_angle: 角度補正の有無
// - angle_min_dot: 角度補正の最小 dot 値
// 戻り値: 補正後の強度 [0,1]
double compensateIntensity(double intensity, const PointXYZ &point,
                           const Eigen::Vector3d &sensor_origin,
                           const Eigen::Vector3d &sensor_forward,
                           double distance_ref, double distance_power,
                           bool compensate_distance, bool compensate_angle,
                           double angle_min_dot);

} // namespace obstacle_detector
