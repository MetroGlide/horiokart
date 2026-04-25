# horiokart_slam

This package contains utilities for posegraph serialization and GNSS-constrained optimization PoCs.

PoC components:
- `src/posegraph_serializer/service_adapter.cpp` : Approach A ServiceAdapter PoC that calls slam_toolbox SerializePoseGraph service and writes a minimal JSON to `/tmp/posegraph_dump.json`.
- `tools/poc_optimize.py` : Lightweight Python PoC optimizer. Example: `python3 tools/poc_optimize.py` (uses feature fixtures).

Building the C++ PoC (ROS 2 workspace):
- Use colcon build in the workspace root. Ensure dependencies (`rclcpp`, `nlohmann_json`) are available.
Build & test instructions (developer):

1. Install dependencies with rosdep from the ROS2 workspace root:

```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

2. Build and test only this package to speed iteration:

```bash
cd /root/ros2_ws
colcon build --packages-select horiokart_slam
colcon test --packages-select horiokart_slam
```

Notes:
- Run builds and tests from `/root/ros2_ws` as required by project conventions.
- If you need to run the service adapter node, source the workspace install before running.

Running Python PoC:
```
/bin/python3 tools/poc_optimize.py
```

Generated artifacts:
- `/tmp/posegraph_dump.json` (service adapter)
- `/tmp/posegraph_optimized.json` (python PoC)
