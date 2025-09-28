#include "horiokart_obstacle_detector_3d/core/cluster_detector.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <cmath>
#include <tuple>
#include <unordered_map>

namespace obstacle_detector {

void ClusterDetector::setParameters(double cluster_tolerance,
                                    int min_cluster_size,
                                    int max_cluster_size) {
  cluster_tolerance_ = cluster_tolerance;
  min_cluster_size_ = min_cluster_size;
  max_cluster_size_ = max_cluster_size;
}

void ClusterDetector::setDownsampleLeafSize(double leaf_size) {
  voxel_leaf_size_ = leaf_size;
}

void ClusterDetector::setOutlierRadius(double radius) {
  outlier_radius_ = radius;
}

void ClusterDetector::setOutlierMinNeighbors(int min_neighbors) {
  outlier_min_neighbors_ = min_neighbors;
}

void ClusterDetector::setPassThroughZ(bool enable, double z_min, double z_max) {
  passthrough_z_enable_ = enable;
  passthrough_z_min_ = z_min;
  passthrough_z_max_ = z_max;
}

void ClusterDetector::setExpandToOriginalCloud(bool enable) {
  expand_to_original_cloud_ = enable;
}

std::vector<Cluster>
ClusterDetector::extractClusters(const std::vector<PointXYZ> &points) const {
  std::vector<obstacle_detector::Cluster> out;
  if (points.empty()) {
    return out;
  }

  // convert to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(points.size());
  // process_cloud is the cloud that will be used for clustering (may be
  // downsampled/filtered). Initialize to the original cloud.
  pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud = cloud;
  for (const auto &p : points) {
    // convert obstacle_detector::PointXYZ -> pcl::PointXYZ
    cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x),
                                   static_cast<float>(p.y),
                                   static_cast<float>(p.z)));
  }
  // keep an index mapping from process_cloud -> original cloud when
  // downsampling if non-empty, voxel_index_map[i] contains indices into 'cloud'
  // that map to process_cloud->points[i]
  std::vector<std::vector<int>> voxel_index_map;
  if (voxel_leaf_size_ > 0.0) {
    // If no further preprocessing is requested, build our own voxelization so
    // we can preserve mapping
    if (!passthrough_z_enable_ && outlier_radius_ <= 0.0) {
      struct VoxelKey {
        int64_t x, y, z;
        bool operator==(const VoxelKey &o) const noexcept {
          return x == o.x && y == o.y && z == o.z;
        }
      };
      struct VoxelKeyHash {
        std::size_t operator()(VoxelKey const &k) const noexcept {
          // combine hashes
          auto h1 = std::hash<int64_t>()(k.x);
          auto h2 = std::hash<int64_t>()(k.y);
          auto h3 = std::hash<int64_t>()(k.z);
          return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
      };

      std::unordered_map<VoxelKey, int, VoxelKeyHash> voxel_to_idx;
      std::vector<pcl::PointXYZ> centroids;
      std::vector<int> counts;
      std::vector<std::vector<int>> temp_indices;
      const double leaf = voxel_leaf_size_;
      for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto &p = cloud->points[i];
        int64_t ix = static_cast<int64_t>(std::floor(p.x / leaf));
        int64_t iy = static_cast<int64_t>(std::floor(p.y / leaf));
        int64_t iz = static_cast<int64_t>(std::floor(p.z / leaf));
        VoxelKey key{ix, iy, iz};
        auto it = voxel_to_idx.find(key);
        if (it == voxel_to_idx.end()) {
          int new_idx = static_cast<int>(centroids.size());
          voxel_to_idx.emplace(key, new_idx);
          centroids.push_back(pcl::PointXYZ(p.x, p.y, p.z));
          counts.push_back(1);
          temp_indices.emplace_back();
          temp_indices.back().push_back(static_cast<int>(i));
        } else {
          int vidx = it->second;
          // accumulate centroid
          centroids[vidx].x += p.x;
          centroids[vidx].y += p.y;
          centroids[vidx].z += p.z;
          counts[vidx] += 1;
          temp_indices[vidx].push_back(static_cast<int>(i));
        }
      }
      // finalize centroids and build process_cloud + mapping
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
          new pcl::PointCloud<pcl::PointXYZ>);
      filtered->reserve(centroids.size());
      voxel_index_map.reserve(temp_indices.size());
      for (size_t vi = 0; vi < centroids.size(); ++vi) {
        filtered->push_back(
            pcl::PointXYZ(static_cast<float>(centroids[vi].x / counts[vi]),
                          static_cast<float>(centroids[vi].y / counts[vi]),
                          static_cast<float>(centroids[vi].z / counts[vi])));
        voxel_index_map.push_back(std::move(temp_indices[vi]));
      }
      process_cloud = filtered;
    } else {
      // fallback to pcl::VoxelGrid when other preprocessing is enabled (no
      // preserved mapping)
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
          new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(voxel_leaf_size_),
                     static_cast<float>(voxel_leaf_size_),
                     static_cast<float>(voxel_leaf_size_));
      vg.filter(*filtered);
      process_cloud = filtered;
      // leave voxel_index_map empty to signal fallback
    }
  }

  // PassThrough Z filter
  if (passthrough_z_enable_) {
    pcl::PassThrough<pcl::PointXYZ> pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pt.setInputCloud(process_cloud);
    pt.setFilterFieldName("z");
    pt.setFilterLimits(static_cast<float>(passthrough_z_min_),
                       static_cast<float>(passthrough_z_max_));
    pt.filter(*filtered);
    process_cloud = filtered;
  }

  // Radius outlier removal
  if (outlier_radius_ > 0.0) {
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    ror.setInputCloud(process_cloud);
    ror.setRadiusSearch(static_cast<float>(outlier_radius_));
    ror.setMinNeighborsInRadius(std::max(1, outlier_min_neighbors_));
    ror.filter(*filtered);
    process_cloud = filtered;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(process_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(std::max(1, min_cluster_size_));
  ec.setMaxClusterSize(std::max(1, max_cluster_size_));
  ec.setSearchMethod(tree);
  ec.setInputCloud(process_cloud);
  ec.extract(cluster_indices);

  int id = 0;
  // If expand_to_original_cloud_ is set, we will expand cluster indices to
  // original points by nearest-neighbor lookup
  pcl::KdTreeFLANN<pcl::PointXYZ> orig_tree;
  if (expand_to_original_cloud_) {
    orig_tree.setInputCloud(cloud);
  }
  for (const auto &indices : cluster_indices) {
    Cluster c;
    c.id = id++;
    c.volume = 0.0;
    c.centroid = PointXYZ{0.0f, 0.0f, 0.0f};
    if (indices.indices.empty()) {
      continue;
    }
    double xmin = 1e9, ymin = 1e9, zmin = 1e9, xmax = -1e9, ymax = -1e9,
           zmax = -1e9;
    // gather points (possibly expanded to original cloud)
    if (expand_to_original_cloud_) {
      // If we constructed a voxel_index_map during downsampling, use it for
      // exact expansion
      if (!voxel_index_map.empty()) {
        for (int idx : indices.indices) {
          if (idx >= 0 && static_cast<size_t>(idx) < voxel_index_map.size()) {
            for (int oi : voxel_index_map[idx]) {
              const auto &opt = cloud->points[oi];
              PointXYZ op{static_cast<float>(opt.x), static_cast<float>(opt.y),
                          static_cast<float>(opt.z)};
              c.points.push_back(op);
              c.centroid.x += op.x;
              c.centroid.y += op.y;
              c.centroid.z += op.z;
              xmin = std::min(xmin, static_cast<double>(op.x));
              ymin = std::min(ymin, static_cast<double>(op.y));
              zmin = std::min(zmin, static_cast<double>(op.z));
              xmax = std::max(xmax, static_cast<double>(op.x));
              ymax = std::max(ymax, static_cast<double>(op.y));
              zmax = std::max(zmax, static_cast<double>(op.z));
            }
          }
        }
      } else {
        std::vector<int> nn_indices;
        std::vector<float> nn_dists;
        for (int idx : indices.indices) {
          const auto &pt = process_cloud->points[idx];
          // radius search to collect original points belonging to this voxel
          float search_radius = static_cast<float>(
              voxel_leaf_size_ > 0.0 ? voxel_leaf_size_ * 3.0f : 0.05f);
          if (orig_tree.radiusSearch(pcl::PointXYZ(static_cast<float>(pt.x),
                                                   static_cast<float>(pt.y),
                                                   static_cast<float>(pt.z)),
                                     search_radius, nn_indices, nn_dists) > 0) {
            for (int oi : nn_indices) {
              const auto &opt = cloud->points[oi];
              PointXYZ op{static_cast<float>(opt.x), static_cast<float>(opt.y),
                          static_cast<float>(opt.z)};
              c.points.push_back(op);
              c.centroid.x += op.x;
              c.centroid.y += op.y;
              c.centroid.z += op.z;
              xmin = std::min(xmin, static_cast<double>(op.x));
              ymin = std::min(ymin, static_cast<double>(op.y));
              zmin = std::min(zmin, static_cast<double>(op.z));
              xmax = std::max(xmax, static_cast<double>(op.x));
              ymax = std::max(ymax, static_cast<double>(op.y));
              zmax = std::max(zmax, static_cast<double>(op.z));
            }
          } else {
            PointXYZ p{static_cast<float>(pt.x), static_cast<float>(pt.y),
                       static_cast<float>(pt.z)};
            c.points.push_back(p);
            c.centroid.x += p.x;
            c.centroid.y += p.y;
            c.centroid.z += p.z;
            xmin = std::min(xmin, static_cast<double>(p.x));
            ymin = std::min(ymin, static_cast<double>(p.y));
            zmin = std::min(zmin, static_cast<double>(p.z));
            xmax = std::max(xmax, static_cast<double>(p.x));
            ymax = std::max(ymax, static_cast<double>(p.y));
            zmax = std::max(zmax, static_cast<double>(p.z));
          }
        }
      }
    } else {
      for (int idx : indices.indices) {
        const auto &pt = process_cloud->points[idx];
        PointXYZ p{pt.x, pt.y, pt.z};
        c.points.push_back(p);
        c.centroid.x += p.x;
        c.centroid.y += p.y;
        c.centroid.z += p.z;
        xmin = std::min(xmin, static_cast<double>(p.x));
        ymin = std::min(ymin, static_cast<double>(p.y));
        zmin = std::min(zmin, static_cast<double>(p.z));
        xmax = std::max(xmax, static_cast<double>(p.x));
        ymax = std::max(ymax, static_cast<double>(p.y));
        zmax = std::max(zmax, static_cast<double>(p.z));
      }
    }

    size_t n = c.points.size();
    if (n == 0) {
      continue;
    }
    c.centroid.x /= n;
    c.centroid.y /= n;
    c.centroid.z /= n;
    c.volume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin);
    // filter by size
    if (static_cast<int>(n) < min_cluster_size_ ||
        static_cast<int>(n) > max_cluster_size_) {
      continue;
    }
    out.push_back(std::move(c));
  }
}

} // namespace obstacle_detector
