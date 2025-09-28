# horiokart_obstacle_detector_3d

This package detects obstacles from a 3D point cloud, separates ground/non-ground, and publishes obstacle point clouds and a LaserScan.

Quick start (after building the workspace):

```bash
# launch the node with default params
ros2 launch horiokart_obstacle_detector_3d obstacle_detector.launch.py
```

Important topics:
- Input point cloud: configurable (default `/camera/depth/points`)
- Output obstacle cloud: `/obstacle_points`
- Output scan: `/obstacle_scan`
- Confidence grid map (cells as PointCloud2): `/confidence_map`
- Footprint traversable boolean: `/footprint_traversable`

New features in this branch:
- PCA-based per-cell slope estimation selectable via `slope_method: "pca"`.
- Intensity compensation by distance/angle for more robust confidence blending.

Parameters of interest (add to launch or params YAML):
- `slope_method`: `finite_difference` or `pca` (default `finite_difference`)
- `pca_radius_m`: radius around a cell to collect points for PCA (default 0.15)
- `pca_min_points`: minimum number of points for PCA in a neighborhood (default 10)
- `intensity_compensate_distance`: bool
- `intensity_distance_ref`: reference distance in meters
- `intensity_distance_power`: exponent for distance scaling
- `intensity_compensate_angle`: bool
- `intensity_angle_min_dot`: minimum dot product to consider full intensity

Notes:
- If using `slope_method: pca`, Eigen3 is required. Ensure system has libeigen3-dev installed.
- The node expects point clouds already time-synchronized; TF transforms will be applied if needed.

