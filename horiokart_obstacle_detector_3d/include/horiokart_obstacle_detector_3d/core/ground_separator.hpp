#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

namespace obstacle_detector {
class GroundSeparator {
public:
  GroundSeparator() = default;

  // パラメータ: 地面とみなす最大高さ、許容する最大分散
  void setParameters(double max_ground_height, double max_variance) {
    std::lock_guard<std::mutex> lk(mutex_);
    max_ground_height_ = max_ground_height;
    max_variance_ = max_variance;
  }

  // 傾斜閾値（度）を設定。閾値以下の傾斜は地面と判定されやすい
  void setSlopeThresholdDeg(double deg) {
    std::lock_guard<std::mutex> lk(mutex_);
    slope_threshold_deg_ = deg;
  }

  // 単純な述語: 低い高さかつ低い分散 => 地面
  bool isGround(double height, double variance) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (height <= max_ground_height_) && (variance <= max_variance_);
  }

  // オーバーロード: 局所傾斜（度）も判定に含める
  bool isGround(double height, double variance, double slope_deg) const {
    std::lock_guard<std::mutex> lk(mutex_);
    return (height <= max_ground_height_) && (variance <= max_variance_) &&
           (slope_deg <= slope_threshold_deg_);
  }

  // 傾斜（度）、分散、セル信頼度から複合的な地面スコア（0..1）を計算
  double computeGroundScore(double slope_deg, double variance,
                            double confidence) const {
    std::lock_guard<std::mutex> lk(mutex_);
    double ns = std::min(1.0, slope_deg / slope_threshold_deg_);
    double nv = std::min(1.0, variance / max_variance_);
    // 傾斜と分散は小さいほど良く、confidence は大きいほど良い
    double score = 1.0 - (w_s_ * ns + w_v_ * nv);
    score = score * (w_c_ * confidence + (1.0 - w_c_));
    if (score < 0.0) {
      score = 0.0;
    }
    if (score > 1.0) {
      score = 1.0;
    }
    return score;
  }

  void setScoreWeights(double w_s, double w_v, double w_c) {
    std::lock_guard<std::mutex> lk(mutex_);
    w_s_ = w_s;
    w_v_ = w_v;
    w_c_ = w_c;
  }

  // 地面スコアの平滑化に用いる EMA の alpha
  // と、ヒステリシス用の高/低しきい値を設定
  void setHysteresisParameters(double ema_alpha, double high_thresh,
                               double low_thresh) {
    std::lock_guard<std::mutex> lk(mutex_);
    ema_alpha_ = ema_alpha;
    high_threshold_ = high_thresh;
    low_threshold_ = low_thresh;
  }

  double getEmaAlpha() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return ema_alpha_;
  }

  double getHighThreshold() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return high_threshold_;
  }

  double getLowThreshold() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return low_threshold_;
  }

private:
  mutable std::mutex mutex_;
  double max_ground_height_ = 0.2; // メートル
  double max_variance_ = 0.02;
  double slope_threshold_deg_ = 15.0;
  double w_s_ = 0.45;
  double w_v_ = 0.35;
  double w_c_ = 0.20;
  double ema_alpha_ = 0.3;
  double high_threshold_ = 0.7;
  double low_threshold_ = 0.4;
};

} // namespace obstacle_detector
