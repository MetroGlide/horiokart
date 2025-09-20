#include "horiokart_depth_camera_costmap/core_processor.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <cstdint>

namespace horiokart
{
    namespace depth_camera_costmap
    {

        // simple integer packing for voxel key (assumes coordinates in reasonable range)
        static inline long long packVoxelKey(int vx, int vy, int vz)
        {
            // pack into 21 bits each (signed via bias)
            const long long B = (1LL << 20); // bias
            long long kx = static_cast<long long>(vx) + B;
            long long ky = static_cast<long long>(vy) + B;
            long long kz = static_cast<long long>(vz) + B;
            return (kx << 42) | (ky << 21) | (kz & 0x1FFFFFLL);
        }

        GridCostMap processPointCloud(const std::vector<Point3D> &points, const CoreParams &params)
        {
            GridCostMap grid;
            grid.resolution = params.grid_resolution;

            if (points.empty())
                return grid;

            // 1) Voxel grid downsampling
            std::unordered_map<long long, std::pair<Point3D, int>> voxels;
            voxels.reserve(points.size());
            const float leaf = (params.voxel_size > 0.f) ? params.voxel_size : 0.02f;
            for (const auto &p : points)
            {
                int vx = static_cast<int>(std::floor(p.x / leaf));
                int vy = static_cast<int>(std::floor(p.y / leaf));
                int vz = static_cast<int>(std::floor(p.z / leaf));
                long long key = packVoxelKey(vx, vy, vz);
                auto &ent = voxels[key];
                if (ent.second == 0)
                {
                    ent.first = p;
                    ent.second = 1;
                }
                else
                {
                    // running average
                    ent.first.x = (ent.first.x * ent.second + p.x) / (ent.second + 1);
                    ent.first.y = (ent.first.y * ent.second + p.y) / (ent.second + 1);
                    ent.first.z = (ent.first.z * ent.second + p.z) / (ent.second + 1);
                    ent.second++;
                }
            }

            std::vector<Point3D> ds_points;
            ds_points.reserve(voxels.size());
            for (auto &kv : voxels)
                ds_points.push_back(kv.second.first);

            // 2) Simple SOR: compute mean distance to k nearest using naive method (O(n^2)), keep those within global threshold
            std::vector<Point3D> filtered = ds_points;
            if (!ds_points.empty())
            {
                int k = std::max(1, params.sor_mean_k);
                const float mul = params.sor_stddev_mul_thresh;
                std::vector<float> mean_dists(ds_points.size(), 0.f);
                for (size_t i = 0; i < ds_points.size(); ++i)
                {
                    std::vector<float> dists;
                    dists.reserve(ds_points.size());
                    for (size_t j = 0; j < ds_points.size(); ++j)
                    {
                        if (i == j)
                            continue;
                        float dx = ds_points[i].x - ds_points[j].x;
                        float dy = ds_points[i].y - ds_points[j].y;
                        float dz = ds_points[i].z - ds_points[j].z;
                        dists.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
                    }
                    if (dists.empty())
                    {
                        mean_dists[i] = 0.f;
                        continue;
                    }
                    std::nth_element(dists.begin(), dists.begin() + std::min<size_t>(k, dists.size()), dists.end());
                    size_t use_k = std::min<size_t>(k, dists.size());
                    double sum = 0.0;
                    for (size_t t = 0; t < use_k; ++t)
                        sum += dists[t];
                    mean_dists[i] = static_cast<float>(sum / static_cast<double>(use_k));
                }
                // global mean/std
                double gmean = 0.0;
                for (float v : mean_dists)
                    gmean += v;
                gmean /= mean_dists.size();
                double var = 0.0;
                for (float v : mean_dists)
                    var += (v - gmean) * (v - gmean);
                var /= mean_dists.size();
                double gstd = std::sqrt(var);
                double thresh = gmean + mul * gstd;
                filtered.clear();
                for (size_t i = 0; i < ds_points.size(); ++i)
                {
                    if (mean_dists[i] <= thresh)
                        filtered.push_back(ds_points[i]);
                }
                if (filtered.empty())
                    filtered = ds_points; // fallback
            }

            // 3) Project to 2D grid and aggregate per-cell
            // determine bounds first
            int min_ix = std::numeric_limits<int>::max();
            int min_iy = std::numeric_limits<int>::max();
            int max_ix = std::numeric_limits<int>::min();
            int max_iy = std::numeric_limits<int>::min();
            for (const auto &p : filtered)
            {
                int ix = static_cast<int>(std::floor(p.x / grid.resolution));
                int iy = static_cast<int>(std::floor(p.y / grid.resolution));
                min_ix = std::min(min_ix, ix);
                min_iy = std::min(min_iy, iy);
                max_ix = std::max(max_ix, ix);
                max_iy = std::max(max_iy, iy);
            }
            if (min_ix > max_ix || min_iy > max_iy)
                return grid; // no valid cells

            grid.width = static_cast<std::uint32_t>(max_ix - min_ix + 1);
            grid.height = static_cast<std::uint32_t>(max_iy - min_iy + 1);
            grid.origin.x = static_cast<float>(min_ix) * grid.resolution;
            grid.origin.y = static_cast<float>(min_iy) * grid.resolution;
            grid.origin.z = 0.f;
            grid.cells.assign(static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height), GridCellFeature());

            // aggregate
            for (const auto &p : filtered)
            {
                int ix = static_cast<int>(std::floor(p.x / grid.resolution)) - min_ix;
                int iy = static_cast<int>(std::floor(p.y / grid.resolution)) - min_iy;
                if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.width) || iy >= static_cast<int>(grid.height))
                    continue;
                size_t idx = grid.index(static_cast<std::uint32_t>(ix), static_cast<std::uint32_t>(iy));
                GridCellFeature &f = grid.cells[idx];
                // Welford for mean/variance z
                f.count += 1;
                float z = p.z;
                if (f.count == 1)
                {
                    f.mean_z = z;
                    f.var_z = 0.f;
                }
                else
                {
                    float delta = z - f.mean_z;
                    f.mean_z += delta / static_cast<float>(f.count);
                    float delta2 = z - f.mean_z;
                    f.var_z = ((static_cast<float>(f.count - 1) * f.var_z) + delta * delta2) / static_cast<float>(f.count);
                }
                // simple normal placeholder: keep upward normal
                f.normal = {0.f, 0.f, 1.f};
            }

            // 4) Assign cost / traversability
            for (size_t i = 0; i < grid.cells.size(); ++i)
            {
                GridCellFeature &f = grid.cells[i];
                if (f.count == 0)
                {
                    f.cost = 255; // unknown
                    f.traversability = 0.f;
                    continue;
                }
                if (f.var_z > params.z_variance_threshold)
                {
                    f.traversability = 0.f;
                    f.cost = params.cost_lethal;
                }
                else
                {
                    f.traversability = 1.f;
                    // safe cost 0 by convention
                    f.cost = 0;
                }
            }

            return grid;
        }

        std::vector<ObstacleCluster> clusterCostMap(const GridCostMap &grid, const ClusterParams &params)
        {
            std::vector<ObstacleCluster> out;
            if (grid.width == 0 || grid.height == 0)
                return out;

            // threshold: consider cell occupied if cost != 255 and cost > 0
            auto is_occupied = [&](std::size_t ix, std::size_t iy) -> bool
            {
                size_t idx = grid.index(static_cast<std::uint32_t>(ix), static_cast<std::uint32_t>(iy));
                const GridCellFeature &f = grid.cells[idx];
                return (f.count > 0) && (f.cost != 255) && (f.cost > 0);
            };

            std::vector<int> labels(grid.width * grid.height, -1);
            int next_label = 0;
            int tol_cells = std::max<int>(1, static_cast<int>(std::ceil(params.cluster_tolerance / grid.resolution)));

            auto idx1d = [&](int x, int y)
            { return y * static_cast<int>(grid.width) + x; };

            for (int y = 0; y < static_cast<int>(grid.height); ++y)
            {
                for (int x = 0; x < static_cast<int>(grid.width); ++x)
                {
                    int id = idx1d(x, y);
                    if (labels[id] != -1)
                        continue;
                    if (!is_occupied(x, y))
                    {
                        labels[id] = -2; // empty
                        continue;
                    }
                    // BFS
                    std::vector<std::pair<int, int>> stack;
                    stack.emplace_back(x, y);
                    labels[id] = next_label;
                    size_t ptr = 0;
                    while (ptr < stack.size())
                    {
                        auto [cx, cy] = stack[ptr++];
                        // explore neighbors within tol_cells (square window)
                        for (int oy = -tol_cells; oy <= tol_cells; ++oy)
                        {
                            for (int ox = -tol_cells; ox <= tol_cells; ++ox)
                            {
                                int nx = cx + ox;
                                int ny = cy + oy;
                                if (nx < 0 || ny < 0 || nx >= static_cast<int>(grid.width) || ny >= static_cast<int>(grid.height))
                                    continue;
                                int nid = idx1d(nx, ny);
                                if (labels[nid] == -1 && is_occupied(nx, ny))
                                {
                                    labels[nid] = next_label;
                                    stack.emplace_back(nx, ny);
                                }
                            }
                        }
                    }
                    // collect cluster
                    std::vector<std::pair<int, int>> cells;
                    for (int yy = 0; yy < static_cast<int>(grid.height); ++yy)
                    {
                        for (int xx = 0; xx < static_cast<int>(grid.width); ++xx)
                        {
                            int cid = idx1d(xx, yy);
                            if (labels[cid] == next_label)
                                cells.emplace_back(xx, yy);
                        }
                    }
                    if (cells.size() >= params.min_cluster_size)
                    {
                        ObstacleCluster c;
                        c.id = static_cast<std::uint64_t>(out.size() + 1);
                        // centroid in world coords
                        double sx = 0.0, sy = 0.0;
                        for (auto &cc : cells)
                        {
                            double wx = grid.origin.x + (static_cast<double>(cc.first) + 0.5) * grid.resolution;
                            double wy = grid.origin.y + (static_cast<double>(cc.second) + 0.5) * grid.resolution;
                            sx += wx;
                            sy += wy;
                            // push approximate point (z = mean_z of cell)
                            const GridCellFeature &gf = grid.cells[grid.index(static_cast<std::uint32_t>(cc.first), static_cast<std::uint32_t>(cc.second))];
                            Point3D pt{static_cast<float>(wx), static_cast<float>(wy), gf.mean_z};
                            c.points.push_back(pt);
                        }
                        c.centroid.x = static_cast<float>(sx / static_cast<double>(cells.size()));
                        c.centroid.y = static_cast<float>(sy / static_cast<double>(cells.size()));
                        // simple type heuristic
                        if (cells.size() > params.min_cluster_size * 5)
                            c.type = ObstacleType::WALL;
                        else
                            c.type = ObstacleType::POINT;
                        c.confidence = 1.0f;
                        out.push_back(std::move(c));
                    }
                    next_label++;
                }
            }

            return out;
        }

        GridCostMap mergeCostMaps(const std::vector<GridCostMap> &maps, const CoreParams &params)
        {
            if (maps.empty())
                return GridCostMap();
            // naive: require same resolution/size/origin; otherwise return first
            GridCostMap base = maps.front();
            for (size_t m = 1; m < maps.size(); ++m)
            {
                const GridCostMap &g = maps[m];
                if (g.width != base.width || g.height != base.height || std::abs(g.resolution - base.resolution) > 1e-6 || std::abs(g.origin.x - base.origin.x) > 1e-6 || std::abs(g.origin.y - base.origin.y) > 1e-6)
                {
                    // incompatible map, skip
                    continue;
                }
                for (size_t i = 0; i < base.cells.size() && i < g.cells.size(); ++i)
                {
                    // take max cost (conservative)
                    uint8_t a = base.cells[i].cost;
                    uint8_t b = g.cells[i].cost;
                    if (a == 255)
                        base.cells[i].cost = b;
                    else if (b == 255)
                        ; // keep a
                    else
                        base.cells[i].cost = std::max(a, b);
                }
            }
            return base;
        }

        std::vector<std::uint8_t> convertToOccupancyArray(const GridCostMap &grid)
        {
            std::vector<std::uint8_t> out;
            if (grid.width == 0 || grid.height == 0)
                return out;
            out.assign(static_cast<size_t>(grid.width) * static_cast<size_t>(grid.height), 255);
            for (std::uint32_t iy = 0; iy < grid.height; ++iy)
            {
                for (std::uint32_t ix = 0; ix < grid.width; ++ix)
                {
                    size_t idx = grid.index(ix, iy);
                    const GridCellFeature &f = grid.cells[idx];
                    out[idx] = f.cost;
                }
            }
            return out;
        }

    } // namespace depth_camera_costmap
} // namespace horiokart
