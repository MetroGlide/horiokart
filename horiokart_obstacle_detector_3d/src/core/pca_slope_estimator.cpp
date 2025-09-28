#include "horiokart_obstacle_detector_3d/core/pca_slope_estimator.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace obstacle_detector {

std::unordered_map<int, std::pair<double, Eigen::Vector3d>>
computePcaSlopesAndNormals(const std::vector<PointXYZ> &pts,
                           const GridHeightMap &grid, double pca_radius_m,
                           int pca_min_points, double grid_cell_size,
                           double roi_x_min, double roi_y_min) {
  std::unordered_map<int, std::pair<double, Eigen::Vector3d>> out;
  if (pts.empty()) {
    return out;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->width = static_cast<uint32_t>(pts.size());
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = false;
  pcl_cloud->points.resize(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    pcl_cloud->points[i].x = pts[i].x;
    pcl_cloud->points[i].y = pts[i].y;
    pcl_cloud->points[i].z = pts[i].z;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(pcl_cloud);

  int rows = grid.rows();
  int cols = grid.cols();
  double r = pca_radius_m;
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;

  for (int ix = 0; ix < rows; ++ix) {
    for (int iy = 0; iy < cols; ++iy) {
      double cx = roi_x_min + (ix + 0.5) * grid_cell_size;
      double cy = roi_y_min + (iy + 0.5) * grid_cell_size;
      pcl::PointXYZ search_point;
      search_point.x = static_cast<float>(cx);
      search_point.y = static_cast<float>(cy);
      search_point.z = 0.0f;
      nn_indices.clear();
      nn_dists.clear();
      int found = kdtree.radiusSearch(search_point, static_cast<double>(r),
                                      nn_indices, nn_dists);
      if (found < pca_min_points) {
        continue;
      }
      Eigen::Vector3d mu(0.0, 0.0, 0.0);
      std::vector<Eigen::Vector3d> samples;
      samples.reserve(nn_indices.size());
      for (int idx : nn_indices) {
        const auto &pp = pcl_cloud->points[idx];
        Eigen::Vector3d s(pp.x, pp.y, pp.z);
        samples.push_back(s);
        mu += s;
      }
      mu /= static_cast<double>(samples.size());
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (const auto &s : samples) {
        Eigen::Vector3d d = s - mu;
        cov += d * d.transpose();
      }
      cov /= static_cast<double>(samples.size());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      if (es.info() != Eigen::Success) {
        continue;
      }
      Eigen::Vector3d normal = es.eigenvectors().col(0);
      normal.normalize();
      double cos_z = std::abs(normal.z());
      double slope_rad = std::acos(std::min(1.0, std::max(-1.0, cos_z)));
      double slope_deg = slope_rad * (180.0 / M_PI);
      out.emplace(ix * cols + iy, std::make_pair(slope_deg, normal));
    }
  }
  return out;
}

} // namespace obstacle_detector
