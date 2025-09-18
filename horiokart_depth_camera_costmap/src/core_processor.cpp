#include "horiokart_depth_camera_costmap/core_processor.hpp"
#include "horiokart_depth_camera_costmap/core_types.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <Eigen/Eigenvalues>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

namespace horiokart::depth_camera_costmap
{

    static inline int floorDiv(double v, double res)
    {
        return static_cast<int>(std::floor(v / res));
    }

    GridCostMap processPointCloud(const std::vector<Point3D> &points, const CoreParams &params)
    {
        GridCostMap grid;
        grid.resolution_m = params.grid_resolution_m;

        if (points.empty())
            return grid;

        // 1) VoxelGrid downsample (3D voxel)
        std::unordered_map<long long, std::pair<Point3D, int>> voxel_map;
        voxel_map.reserve(points.size() / 4 + 1);
        const double leaf = params.voxel_leaf_size_m > 0.0 ? params.voxel_leaf_size_m : 0.02;
        for (const auto &p : points)
        {
            int vx = static_cast<int>(std::floor(p.x / leaf));
            int vy = static_cast<int>(std::floor(p.y / leaf));
            int vz = static_cast<int>(std::floor(p.z / leaf));
            long long key = ((static_cast<long long>(vx) & 0x1FFFFF) << 42) |
                            ((static_cast<long long>(vy) & 0x1FFFFF) << 21) |
                            (static_cast<long long>(vz) & 0x1FFFFF);
            auto &entry = voxel_map[key];
            if (entry.second == 0)
                entry.first = p;
            else
            {
                // accumulate average
                entry.first.x = (entry.first.x * entry.second + p.x) / (entry.second + 1);
                entry.first.y = (entry.first.y * entry.second + p.y) / (entry.second + 1);
                entry.first.z = (entry.first.z * entry.second + p.z) / (entry.second + 1);
                entry.first.r = static_cast<uint8_t>((entry.first.r * entry.second + p.r) / (entry.second + 1));
                entry.first.g = static_cast<uint8_t>((entry.first.g * entry.second + p.g) / (entry.second + 1));
                entry.first.b = static_cast<uint8_t>((entry.first.b * entry.second + p.b) / (entry.second + 1));
            }
            entry.second += 1;
        }

        std::vector<Point3D> ds_points;
        ds_points.reserve(voxel_map.size());
        for (auto &kv : voxel_map)
            ds_points.push_back(kv.second.first);

        // 2) Statistical Outlier Removal (k-d tree based)
        std::vector<Point3D> filtered_points;
        if (!ds_points.empty())
        {
            int k = std::max(1, params.sor_mean_k);
            // build PCL cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->reserve(ds_points.size());
            for (const auto &p : ds_points)
                cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud);

            std::vector<double> mean_dists(ds_points.size(), 0.0);
            std::vector<int> pointIdxNKNSearch(k);
            std::vector<float> pointNKNSquaredDistance(k);

            for (size_t i = 0; i < ds_points.size(); ++i)
            {
                if (kdtree.nearestKSearch(cloud->points[i], k + 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                    // skip the first result if it's the point itself (distance 0)
                    int start = 0;
                    if (!pointNKNSquaredDistance.empty() && pointNKNSquaredDistance[0] < 1e-12f)
                        start = 1;
                    double sum = 0.0;
                    int cnt = 0;
                    for (size_t t = start; t < pointNKNSquaredDistance.size(); ++t)
                    {
                        sum += std::sqrt(pointNKNSquaredDistance[t]);
                        cnt++;
                    }
                    if (cnt > 0)
                        mean_dists[i] = sum / cnt;
                    else
                        mean_dists[i] = 0.0;
                }
                else
                    mean_dists[i] = 0.0;
            }
            // compute global mean and stddev
            double gmean = 0.0;
            for (double v : mean_dists)
                gmean += v;
            gmean /= mean_dists.size();
            double var = 0.0;
            for (double v : mean_dists)
                var += (v - gmean) * (v - gmean);
            var /= mean_dists.size();
            double gstd = std::sqrt(var);
            double thresh = gmean + params.sor_stddev_mul_thresh * gstd;
            for (size_t i = 0; i < ds_points.size(); ++i)
            {
                if (mean_dists[i] <= thresh)
                    filtered_points.push_back(ds_points[i]);
            }
        }

        if (filtered_points.empty())
            filtered_points = ds_points; // fallback

        // 3) Project to 2D grid cells and aggregate per-cell points for feature computation
        std::unordered_map<std::pair<int, int>, std::vector<Point3D>, PairHash> cell_points;
        for (const auto &p : filtered_points)
        {
            int ix = floorDiv(p.x, grid.resolution_m);
            int iy = floorDiv(p.y, grid.resolution_m);
            cell_points[std::make_pair(ix, iy)].push_back(p);
        }

        // determine bounds
        bool first = true;
        for (const auto &kv : cell_points)
        {
            int ix = kv.first.first;
            int iy = kv.first.second;
            if (first)
            {
                grid.min_ix = grid.max_ix = ix;
                grid.min_iy = grid.max_iy = iy;
                first = false;
            }
            else
            {
                grid.min_ix = std::min(grid.min_ix, ix);
                grid.max_ix = std::max(grid.max_ix, ix);
                grid.min_iy = std::min(grid.min_iy, iy);
                grid.max_iy = std::max(grid.max_iy, iy);
            }
        }
        if (!first)
        {
            grid.width = grid.max_ix - grid.min_ix + 1;
            grid.height = grid.max_iy - grid.min_iy + 1;
            grid.origin.x = static_cast<double>(grid.min_ix) * grid.resolution_m;
            grid.origin.y = static_cast<double>(grid.min_iy) * grid.resolution_m;
            grid.origin.z = 0.0;
        }

        // 4) Compute per-cell features (z stats, mean_rgb, normal via PCA)
        for (const auto &kv : cell_points)
        {
            const auto &pts = kv.second;
            GridCellFeature f;
            f.count = 0;
            // Welford for z
            double mean = 0.0, m2 = 0.0;
            float zmin = std::numeric_limits<float>::infinity();
            float zmax = -std::numeric_limits<float>::infinity();
            Eigen::Vector3f rgb_sum(0.f, 0.f, 0.f);
            // centroid for covariance
            Eigen::Vector3f centroid(0.f, 0.f, 0.f);
            for (const auto &p : pts)
            {
                centroid += Eigen::Vector3f(p.x, p.y, p.z);
            }
            centroid /= static_cast<float>(pts.size());
            Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
            for (const auto &p : pts)
            {
                double x = p.x - centroid.x();
                double y = p.y - centroid.y();
                double z = p.z - centroid.z();
                cov(0, 0) += x * x;
                cov(0, 1) += x * y;
                cov(0, 2) += x * z;
                cov(1, 0) += y * x;
                cov(1, 1) += y * y;
                cov(1, 2) += y * z;
                cov(2, 0) += z * x;
                cov(2, 1) += z * y;
                cov(2, 2) += z * z;
                // z stats
                f.count += 1;
                if (p.z < zmin)
                    zmin = p.z;
                if (p.z > zmax)
                    zmax = p.z;
                double delta = p.z - mean;
                mean += delta / f.count;
                double delta2 = p.z - mean;
                m2 += delta * delta2;
                rgb_sum += Eigen::Vector3f(p.r, p.g, p.b);
            }
            f.z_min = zmin;
            f.z_max = zmax;
            f.z_mean = static_cast<float>(mean);
            f.z_m2 = static_cast<float>(m2);
            f.z_variance = (f.count > 1) ? static_cast<float>(m2 / (f.count - 1)) : 0.f;
            f.mean_rgb = rgb_sum / static_cast<float>(f.count);
            // covariance normalization
            cov /= static_cast<float>(pts.size());
            // eigen decomposition for normal (smallest eigenvalue)
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
            if (solver.info() == Eigen::Success)
            {
                Eigen::Vector3f eigvals = solver.eigenvalues();
                Eigen::Matrix3f eigvecs = solver.eigenvectors();
                // eigenvalues are ordered ascending; normal = eigenvector for smallest eigenvalue
                Eigen::Vector3f normal = eigvecs.col(0);
                if (normal.norm() > 0)
                    normal.normalize();
                f.mean_normal = normal;
            }
            else
            {
                f.mean_normal = Eigen::Vector3f(0.f, 0.f, 1.f);
            }
            // store feature
            // also assign traversability cost according to simplified rules
            float dz = f.z_max - f.z_min;
            uint8_t cost = 0;
            if (dz > params.max_step_height_m)
                cost = params.cost_obstacle;
            else if (f.z_mean > (params.max_step_height_m * 0.5) && f.z_mean > 0.1)
                cost = params.cost_obstacle;
            else if (f.z_variance > params.z_variance_threshold)
                cost = params.cost_semi_traversable;
            else
                cost = params.cost_traversable;
            GridCellFeature stored = f;
            grid.costs[kv.first] = cost;
        }

        return grid;
    }

    std::vector<ObstacleCluster> clusterCostMap(const GridCostMap &grid, const ClusterParams &params)
    {
        std::vector<ObstacleCluster> out;
        // collect points (cells) with cost >= threshold
        std::vector<std::pair<int, int>> pts;
        for (const auto &kv : grid.costs)
        {
            if (kv.second >= params.cost_threshold)
                pts.push_back(kv.first);
        }
        if (pts.empty())
            return out;

        int n = static_cast<int>(pts.size());
        std::vector<int> labels(n, -1); // -1 unvisited
        int cluster_id = 0;
        double eps_cells = params.cluster_distance_threshold_m / grid.resolution_m;

        auto dist2 = [&](int a, int b)
        {
            double dx = pts[a].first - pts[b].first;
            double dy = pts[a].second - pts[b].second;
            return dx * dx + dy * dy;
        };

        for (int i = 0; i < n; ++i)
        {
            if (labels[i] != -1)
                continue;
            // find neighbors
            std::vector<int> nb;
            for (int j = 0; j < n; ++j)
            {
                if (std::sqrt(dist2(i, j)) <= eps_cells)
                    nb.push_back(j);
            }
            if ((int)nb.size() < params.cluster_min_points)
            {
                labels[i] = -2; // noise
                continue;
            }
            // expand
            std::vector<int> stack = nb;
            labels[i] = cluster_id;
            while (!stack.empty())
            {
                int idx = stack.back();
                stack.pop_back();
                if (labels[idx] == -2)
                    labels[idx] = cluster_id;
                if (labels[idx] != -1 && labels[idx] != -2 && labels[idx] != cluster_id)
                    continue;
                labels[idx] = cluster_id;
                // neighbors of idx
                for (int j = 0; j < n; ++j)
                {
                    if (labels[j] != -1)
                        continue;
                    if (std::sqrt(dist2(idx, j)) <= eps_cells)
                    {
                        stack.push_back(j);
                        labels[j] = cluster_id;
                    }
                }
            }
            cluster_id++;
        }

        // collect clusters
        for (int cid = 0; cid < cluster_id; ++cid)
        {
            ObstacleCluster c;
            for (int i = 0; i < n; ++i)
                if (labels[i] == cid)
                    c.cells.push_back(pts[i]);
            if ((int)c.cells.size() < params.cluster_min_points)
                continue;
            // centroid
            Eigen::Vector2f sum(0.f, 0.f);
            for (const auto &cell : c.cells)
                sum += Eigen::Vector2f((float)cell.first, (float)cell.second);
            c.centroid = sum / static_cast<float>(c.cells.size());
            // simple type heuristic
            if ((int)c.cells.size() > 20)
                c.type = ObstacleCluster::Type::WALL;
            else if ((int)c.cells.size() > 5)
                c.type = ObstacleCluster::Type::ROCK;
            else
                c.type = ObstacleCluster::Type::UNKNOWN;
            out.push_back(std::move(c));
        }

        return out;
    }

    GridCostMap mergeCostMaps(const GridCostMap &a, const GridCostMap &b, MergeMode m)
    {
        GridCostMap r = a;
        r.resolution_m = a.resolution_m;
        for (const auto &kv : b.costs)
        {
            auto it = r.costs.find(kv.first);
            if (it == r.costs.end())
                r.costs[kv.first] = kv.second;
            else
            {
                if (m == MergeMode::OVERWRITE_MAX)
                    it->second = std::max(it->second, kv.second);
                else
                    it->second = kv.second;
            }
        }
        return r;
    }

    std::vector<uint8_t> convertToOccupancyArray(const GridCostMap &grid, const OccupancyOptions &opt)
    {
        std::vector<uint8_t> out;
        if (grid.width <= 0 || grid.height <= 0)
            return out;
        out.assign(grid.width * grid.height, 255); // default unknown
        for (const auto &kv : grid.costs)
        {
            int ix = kv.first.first - grid.min_ix;
            int iy = kv.first.second - grid.min_iy;
            if (ix < 0 || ix >= grid.width || iy < 0 || iy >= grid.height)
                continue;
            int idx = iy * grid.width + ix; // row-major
            out[idx] = kv.second;           // 0..254 valid, 255 unknown
        }
        return out;
    }

} // namespace
