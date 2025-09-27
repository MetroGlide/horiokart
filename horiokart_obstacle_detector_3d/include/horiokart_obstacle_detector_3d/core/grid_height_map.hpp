#pragma once

#include <mutex>
#include <vector>

#include "horiokart_obstacle_detector_3d/core/types.hpp"

namespace obstacle_detector
{
class GridHeightMap
{
public:
  GridHeightMap() = default;
  GridHeightMap(double x_min, double x_max, double y_min, double y_max, double cell_size);

  void reset();
  void accumulatePoint(const PointXYZ & p);
  void finalizeFrame(double now_sec);

  // 補間 / 時系列融合 / タイムアウト用パラメータ
  void setParameters(
    int min_obs_per_cell_for_confident_median, int radius_interp_cells, double interp_power_p,
    double interp_alpha, double max_interp_area_m2, double temporal_alpha_height,
    double temporal_alpha_conf, double observation_timeout);

  bool getCell(int ix, int iy, GridCell & out) const;
  // セル (ix,iy) の局所傾斜（度）を返す。範囲外または観測なしの場合は false を返す
  bool getCellSlopeDeg(int ix, int iy, double & out_slope_deg) const;

  int rows() const { return rows_; }
  int cols() const { return cols_; }

private:
  // 補間および時系列融合のパラメータ
  int min_obs_per_cell_for_confident_median_ = 3;
  int radius_interp_cells_ = 3;
  double interp_power_p_ = 2.0;
  double interp_alpha_ = 0.6;
  double max_interp_area_m2_ = 0.5;
  double temporal_alpha_height_ = 0.3;
  double temporal_alpha_conf_ = 0.4;
  double observation_timeout_ = 0.5;  // 秒

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_min_ = 0.0;
  double y_max_ = 0.0;
  double cell_size_ = 0.05;

  int rows_ = 0;
  int cols_ = 0;

  mutable std::mutex mutex_;
  std::vector<std::vector<std::vector<double>>> cell_buffers_;  // セルごとの z 値リスト
  std::vector<GridCell> cells_;                                 // 行優先: ix * cols_ + iy
  // セルごとの局所傾斜（度、ヒートマップから算出）
  std::vector<double> cell_slope_deg_;

  int index(int ix, int iy) const { return ix * cols_ + iy; }
};

}  // namespace obstacle_detector
