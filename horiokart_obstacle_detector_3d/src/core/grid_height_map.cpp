#include "horiokart_obstacle_detector_3d/core/grid_height_map.hpp"

#include <algorithm>
#include <cmath>

using namespace obstacle_detector;

GridHeightMap::GridHeightMap(
  double x_min, double x_max, double y_min, double y_max, double cell_size)
: x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), cell_size_(cell_size)
{
  cols_ = static_cast<int>(std::ceil((y_max_ - y_min_) / cell_size_));
  rows_ = static_cast<int>(std::ceil((x_max_ - x_min_) / cell_size_));
  cell_buffers_.assign(rows_, std::vector<std::vector<double>>(cols_));
  cells_.assign(rows_ * cols_, GridCell());
}

void GridHeightMap::reset()
{
  std::lock_guard<std::mutex> lk(mutex_);
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      cell_buffers_[i][j].clear();
    }
  }
  std::fill(cells_.begin(), cells_.end(), GridCell());
}

void GridHeightMap::accumulatePoint(const PointXYZ & p)
{
  int ix = static_cast<int>(std::floor((p.x - x_min_) / cell_size_));
  int iy = static_cast<int>(std::floor((p.y - y_min_) / cell_size_));
  if (ix < 0 || ix >= rows_ || iy < 0 || iy >= cols_) {
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  cell_buffers_[ix][iy].push_back(p.z);
}

void GridHeightMap::finalizeFrame(double now_sec)
{
  std::lock_guard<std::mutex> lk(mutex_);
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      auto & buf = cell_buffers_[i][j];
      GridCell & cell = cells_[index(i, j)];
      if (buf.empty()) {
        // no new observation: check timeout
        if ((now_sec - cell.last_observed_time) > observation_timeout_) {
          // expire
          cell.has_observation = false;
          cell.confidence = 0.0;
        }
        continue;
      }
      // remember previous state before overwriting
      bool prev_has_observation = cell.has_observation && (cell.last_observed_time > 0.0);
      double prev_height = cell.height_median;
      double prev_conf = cell.confidence;
      cell.obs_count = static_cast<int>(buf.size());
      // compute median
      std::sort(buf.begin(), buf.end());
      const int n = static_cast<int>(buf.size());
      if (n % 2 == 1) {
        cell.height_median = buf[n / 2];
      } else {
        cell.height_median = 0.5 * (buf[n / 2 - 1] + buf[n / 2]);
      }
      // mean and variance
      double sum = 0.0;
      for (double v : buf) {
        sum += v;
      }
      cell.height_mean = sum / n;
      double var = 0.0;
      for (double v : buf) {
        var += (v - cell.height_mean) * (v - cell.height_mean);
      }
      cell.height_variance = var / n;
      // compute confidence baseline
      double new_height = cell.height_median;
      double new_conf = (n >= min_obs_per_cell_for_confident_median_)
                          ? 1.0
                          : (static_cast<double>(n) / min_obs_per_cell_for_confident_median_);
      if (prev_has_observation) {
        // EMA fusion with previous stored values
        cell.height_median =
          temporal_alpha_height_ * new_height + (1.0 - temporal_alpha_height_) * prev_height;
        cell.confidence =
          temporal_alpha_conf_ * new_conf + (1.0 - temporal_alpha_conf_) * prev_conf;
      } else {
        // first observation for this cell in recent history: take new values directly
        cell.height_median = new_height;
        cell.confidence = new_conf;
      }
      cell.has_observation = true;
      cell.last_observed_time = now_sec;
    }
  }

  // compute local slope (finite differences) into cell_slope_deg_
  cell_slope_deg_.assign(rows_ * cols_, 0.0);
  for (int i = 1; i < rows_ - 1; ++i) {
    for (int j = 1; j < cols_ - 1; ++j) {
      GridCell & c = cells_[index(i, j)];
      if (!c.has_observation) {
        continue;
      }
      double h_x1 = cells_[index(i + 1, j)].has_observation ? cells_[index(i + 1, j)].height_median
                                                            : c.height_median;
      double h_x0 = cells_[index(i - 1, j)].has_observation ? cells_[index(i - 1, j)].height_median
                                                            : c.height_median;
      double h_y1 = cells_[index(i, j + 1)].has_observation ? cells_[index(i, j + 1)].height_median
                                                            : c.height_median;
      double h_y0 = cells_[index(i, j - 1)].has_observation ? cells_[index(i, j - 1)].height_median
                                                            : c.height_median;
      double gx = (h_x1 - h_x0) / (2.0 * cell_size_);
      double gy = (h_y1 - h_y0) / (2.0 * cell_size_);
      double slope_rad = std::atan(std::hypot(gx, gy));
      cell_slope_deg_[index(i, j)] = slope_rad * (180.0 / M_PI);
    }
  }

  // interpolate small holes using connected components analysis + IDW
  std::vector<int> comp(rows_ * cols_, -1);
  int comp_id = 0;
  for (int i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols_; ++j) {
      int idx = index(i, j);
      if (comp[idx] != -1) {
        continue;
      }
      if (cells_[idx].has_observation) {
        comp[idx] = -2;  // observed
        continue;
      }
      // BFS to collect this empty region
      std::vector<std::pair<int, int>> stack;
      stack.emplace_back(i, j);
      comp[idx] = comp_id;
      size_t cursor = 0;
      while (cursor < stack.size()) {
        auto [ci, cj] = stack[cursor++];
        const int di[4] = {1, -1, 0, 0};
        const int dj[4] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
          int ni = ci + di[k];
          int nj = cj + dj[k];
          if (ni < 0 || ni >= rows_ || nj < 0 || nj >= cols_) {
            continue;
          }
          int nidx = index(ni, nj);
          if (comp[nidx] != -1) {
            continue;
          }
          if (cells_[nidx].has_observation) {
            comp[nidx] = -2;
            continue;
          }
          comp[nidx] = comp_id;
          stack.emplace_back(ni, nj);
        }
      }
      // now stack holds all cells in this empty component
      double area_m2 = stack.size() * (cell_size_ * cell_size_);
      if (area_m2 <= max_interp_area_m2_) {
        // interpolate each cell in stack using IDW from neighbors within radius_interp_cells_
        for (const auto & pr : stack) {
          int ci = pr.first;
          int cj = pr.second;
          double weight_sum = 0.0;
          double weighted_height = 0.0;
          double conf_sum = 0.0;
          int neighbor_count = 0;
          for (int di = -radius_interp_cells_; di <= radius_interp_cells_; ++di) {
            for (int dj = -radius_interp_cells_; dj <= radius_interp_cells_; ++dj) {
              int ni = ci + di;
              int nj = cj + dj;
              if (ni < 0 || ni >= rows_ || nj < 0 || nj >= cols_) {
                continue;
              }
              GridCell & ncell = cells_[index(ni, nj)];
              if (!ncell.has_observation) {
                continue;
              }
              double dist = std::hypot(di, dj);
              double w = ncell.confidence / (std::pow(dist + 1e-6, interp_power_p_));
              weighted_height += w * ncell.height_median;
              weight_sum += w;
              conf_sum += ncell.confidence;
              neighbor_count++;
            }
          }
          if (neighbor_count == 0) {
            continue;
          }
          double interp_h = weighted_height / weight_sum;
          double interp_conf = (conf_sum / neighbor_count) * interp_alpha_;
          GridCell & cell = cells_[index(ci, cj)];
          cell.height_median = interp_h;
          cell.confidence = interp_conf;
          cell.has_observation = true;  // mark as interpolated
        }
      }
      comp_id++;
    }
  }
}

void GridHeightMap::setParameters(
  int min_obs_per_cell_for_confident_median, int radius_interp_cells, double interp_power_p,
  double interp_alpha, double max_interp_area_m2, double temporal_alpha_height,
  double temporal_alpha_conf, double observation_timeout)
{
  min_obs_per_cell_for_confident_median_ = min_obs_per_cell_for_confident_median;
  radius_interp_cells_ = radius_interp_cells;
  interp_power_p_ = interp_power_p;
  interp_alpha_ = interp_alpha;
  max_interp_area_m2_ = max_interp_area_m2;
  temporal_alpha_height_ = temporal_alpha_height;
  temporal_alpha_conf_ = temporal_alpha_conf;
  observation_timeout_ = observation_timeout;
}

bool GridHeightMap::getCell(int ix, int iy, GridCell & out) const
{
  if (ix < 0 || ix >= rows_ || iy < 0 || iy >= cols_) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  out = cells_[index(ix, iy)];
  return true;
}

bool GridHeightMap::getCellSlopeDeg(int ix, int iy, double & out_slope_deg) const
{
  if (ix < 0 || ix >= rows_ || iy < 0 || iy >= cols_) {
    return false;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  if (cell_slope_deg_.empty()) {
    return false;
  }
  out_slope_deg = cell_slope_deg_[index(ix, iy)];
  return true;
}
