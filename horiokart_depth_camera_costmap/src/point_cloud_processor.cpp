// ファイル: point_cloud_processor.cpp
// 概要: 点群前処理ユーティリティを提供します。
//       - downsample: VoxelGrid によるダウンサンプリング
//       - removeOutliers: StatisticalOutlierRemoval による外れ値除去
//       - transform: Eigen 変換を用いた点群座標変換
//       - computeGridFeatures: 点群をグリッドに投影し，各セルごとに高さや色，法線などの特徴量を算出

#include "horiokart_depth_camera_costmap/point_cloud_processor.hpp"
#include <cmath>
#include <unordered_map>
#include <algorithm>

namespace horiokart
{
    namespace depth_camera_costmap
    {

        PointCloudProcessor::PointCloudProcessor() {}

        static long long packVoxelKeyInt(int vx, int vy, int vz)
        {
            const long long B = (1LL << 20);
            long long kx = static_cast<long long>(vx) + B;
            long long ky = static_cast<long long>(vy) + B;
            long long kz = static_cast<long long>(vz) + B;
            return (kx << 42) | (ky << 21) | (kz & 0x1FFFFFLL);
        }

        std::vector<Point3D> PointCloudProcessor::downsample(const std::vector<Point3D> &points, float leaf_size)
        {
            if (points.empty() || leaf_size <= 0.f)
                return points;
            std::unordered_map<long long, std::pair<Point3D, int>> voxels;
            voxels.reserve(points.size());
            for (const auto &p : points)
            {
                int vx = static_cast<int>(std::floor(p.x / leaf_size));
                int vy = static_cast<int>(std::floor(p.y / leaf_size));
                int vz = static_cast<int>(std::floor(p.z / leaf_size));
                long long key = packVoxelKeyInt(vx, vy, vz);
                auto &ent = voxels[key];
                if (ent.second == 0)
                {
                    ent.first = p;
                    ent.second = 1;
                }
                else
                {
                    ent.first.x = (ent.first.x * ent.second + p.x) / (ent.second + 1);
                    ent.first.y = (ent.first.y * ent.second + p.y) / (ent.second + 1);
                    ent.first.z = (ent.first.z * ent.second + p.z) / (ent.second + 1);
                    ent.second++;
                }
            }
            std::vector<Point3D> out;
            out.reserve(voxels.size());
            for (auto &kv : voxels)
                out.push_back(kv.second.first);
            return out;
        }

        std::vector<Point3D> PointCloudProcessor::removeOutliers(const std::vector<Point3D> &points, int mean_k, double stddev_mul_thresh)
        {
            if (points.empty() || mean_k <= 0)
                return points;
            size_t n = points.size();
            std::vector<double> mean_dists(n, 0.0);
            for (size_t i = 0; i < n; ++i)
            {
                std::vector<double> dists;
                dists.reserve(n - 1);
                for (size_t j = 0; j < n; ++j)
                {
                    if (i == j)
                        continue;
                    double dx = points[i].x - points[j].x;
                    double dy = points[i].y - points[j].y;
                    double dz = points[i].z - points[j].z;
                    dists.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
                }
                if (dists.empty())
                    mean_dists[i] = 0.0;
                else
                {
                    std::nth_element(dists.begin(), dists.begin() + std::min<size_t>(dists.size() - 1, static_cast<size_t>(mean_k)), dists.end());
                    size_t use_k = std::min<size_t>(dists.size(), static_cast<size_t>(mean_k));
                    double sum = 0.0;
                    for (size_t t = 0; t < use_k; ++t)
                        sum += dists[t];
                    mean_dists[i] = sum / static_cast<double>(use_k);
                }
            }
            double gmean = 0.0;
            for (double v : mean_dists)
                gmean += v;
            gmean /= mean_dists.size();
            double var = 0.0;
            for (double v : mean_dists)
                var += (v - gmean) * (v - gmean);
            var /= mean_dists.size();
            double gstd = std::sqrt(var);
            double thresh = gmean + stddev_mul_thresh * gstd;
            std::vector<Point3D> out;
            out.reserve(n);
            for (size_t i = 0; i < n; ++i)
            {
                if (mean_dists[i] <= thresh)
                    out.push_back(points[i]);
            }
            if (out.empty())
                return points;
            return out;
        }

        std::map<std::pair<int, int>, GridCellFeature> PointCloudProcessor::computeGridFeatures(const std::vector<Point3D> &points, float grid_resolution)
        {
            std::map<std::pair<int, int>, std::vector<Point3D>> gridmap;
            for (const auto &p : points)
            {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                    continue;
                int ix = static_cast<int>(std::floor(p.x / grid_resolution));
                int iy = static_cast<int>(std::floor(p.y / grid_resolution));
                gridmap[{ix, iy}].push_back(p);
            }
            std::map<std::pair<int, int>, GridCellFeature> features;
            for (const auto &kv : gridmap)
            {
                const auto &pts = kv.second;
                if (pts.empty())
                    continue;
                GridCellFeature f;
                f.count = 0;
                double mean = 0.0, m2 = 0.0;
                float zmin = std::numeric_limits<float>::infinity();
                float zmax = -std::numeric_limits<float>::infinity();
                for (const auto &p : pts)
                {
                    f.count++;
                    if (p.z < zmin)
                        zmin = p.z;
                    if (p.z > zmax)
                        zmax = p.z;
                    double delta = p.z - mean;
                    mean += delta / f.count;
                    double delta2 = p.z - mean;
                    m2 += delta * delta2;
                }
                f.mean_z = static_cast<float>(mean);
                f.var_z = (f.count > 1) ? static_cast<float>(m2 / (f.count - 1)) : 0.f;
                f.z_min = zmin;
                f.z_max = zmax;
                f.normal = {0.f, 0.f, 1.f};
                features[kv.first] = f;
            }
            return features;
        }

    } // namespace depth_camera_costmap
} // namespace horiokart
