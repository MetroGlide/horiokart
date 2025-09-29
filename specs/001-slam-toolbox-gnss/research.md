# research.md

## Purpose
Phase 0 research notes resolving NEEDS CLARIFICATION items and recording decisions for the planning phase.

## Decisions
- GNSS topic/type: `/fix` :: `sensor_msgs/NavSatFix` (provided by user).
- UTM zone selection: auto-select by median of observed coordinates.
- GNSS accuracy assumption: RTK-level (horizontal ≲ 0.5 m). Outlier handling mandatory.
- Offline constraint: do not modify `slam_toolbox`; use rosbag + posegraph export for external optimization.

## Rationale
- Using `sensor_msgs/NavSatFix` keeps integration with common ROS GNSS stacks.
- Median-based UTM zone selection avoids wrong-zone assignments for routes distant from zone boundaries in typical runs.
- RTK assumption is required to meet NFR registration targets; outlier handling protects against intermittent degradation.

## Open Questions (minor)
- Confirm whether `slam_toolbox` deployment emits a posegraph topic named `graph_visualization` or provides a serialization API; if not, allow a small wrapper node to export posegraph from runtime topics.
- Confirm bench HW for NFR-002 (default: 8 core / 32 GB / NVMe SSD).

## Next steps
1. Generate data model and JSON/YAML schemas for PoseGraph, GNSSObservation, AlignedMap.
2. Create quickstart showing CLI usage for the offline optimizer.
3. Prepare contracts for parsing tools and outputs.

<!-- Feedback appended: prioritize timebase verification and a small PoC toolchain for GNSS extraction/transform/match/optimize. -->
Feedback: before designing matching internals, verify that posegraph timestamps and NavSatFix `header.stamp` share a compatible epoch/units. Implement PoC toolchain to validate ideas quickly.
