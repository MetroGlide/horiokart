# quickstart.md

## Quickstart: Offline GNSS-constrained map alignment

Prerequisites: a rosbag with topics `/scan`, `/tf`, `/odom`, `/fix` (sensor_msgs/NavSatFix) and slam_toolbox outputs (posegraph or graph_visualization).

Example CLI (placeholder):

```
slam_gnss_optimize \
  --bag /path/to/run.bag \
  --gnss-topic /fix \
  --posegraph /path/to/posegraph.json \
  --out-map ./aligned_map \
  --config ./config.yaml
```

Outputs:
- ./aligned_map/map.pgm
- ./aligned_map/map.yaml (includes origin_utm)
- ./aligned_map/posegraph_aligned.json
- ./aligned_map/registration_report.json
