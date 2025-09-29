# Feature Specification: slam_toolbox + GNSS 拘束による地球座標整合 2D 環境地図

**Feature Branch**: `001-slam-toolbox-gnss`  
**Created**: 2025-09-25  
**Status**: Draft  
**Input**: User description: "slam_toolboxをベースにgnssの拘束を含めて地球座標と一致する環境地図を作成する方法の実装"

## Execution Flow (main)
```
1. Parse user description from Input
   → If empty: ERROR "No feature description provided"
2. Extract key concepts from description
   → Identify: actors, actions, data, constraints
3. For each unclear aspect:
   → Mark with [NEEDS CLARIFICATION: specific question]
4. Fill User Scenarios & Testing section
   → If no clear user flow: ERROR "Cannot determine user scenarios"
5. Generate Functional Requirements
   → Each requirement must be testable
   → Mark ambiguous requirements
6. Identify Key Entities (if data involved)
7. Run Review Checklist
   → If any [NEEDS CLARIFICATION]: WARN "Spec has uncertainties"
   → If implementation details found: ERROR "Remove tech details"
8. Return: SUCCESS (spec ready for planning)
```

---

## Clarifications

### Session 1 — ユーザ確定事項 (provided)
- GNSS message 型と topic: topic=`/fix`, 型=`sensor_msgs/NavSatFix`.
- UTM zone の選定方針: 観測の中央値で自動選択する。
- GNSS 精度前提: RTK レベル（水平精度 ≲ 0.5 m）を想定する。ただし外れ値処理は必須とする。
- オフライン処理の前提: `slam_toolbox` のコードは変更しない。全処理は rosbag と `slam_toolbox` の出力 (posegraph) に基づいて外部で実行する。


## ⚡ Quick Guidelines
- ✅ Focus on WHAT users need and WHY
- ❌ Avoid HOW to implement (no tech stack, APIs, code structure)
- 👥 Written for business stakeholders, not developers

### Section Requirements
- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation
When creating this spec from a user prompt:
1. **Mark all ambiguities**: Use [NEEDS CLARIFICATION: specific question] for any assumption you'd need to make
2. **Don't guess**: If the prompt doesn't specify something (e.g., "login system" without auth method), mark it
3. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
4. **Common underspecified areas**:
   - User types and permissions
   - Data retention/deletion policies  
   - Performance targets and scale
   - Error handling behaviors
   - Integration requirements
   - Security/compliance needs

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
ロボット運用者は、屋外の広域（例: 2km 以上）環境において、2次元の環境地図を作成し、
その地図を地球座標系（緯度経度）に整合させたい。SLAM の経路最適化に GNSS 観測を拘束として
組み込み、地図の変形（局所的な歪み）を最小化したうえで、地図座標と地球座標の整合を実現する。

加えて、地図完成後は `amcl` と `robot_localization` を用いて運用中の自己位置推定を行い、
GNSS とローカル自己位置推定の融合により、地図上の自己位置を地球座標系で提供する。

### Acceptance Scenarios
1. **Given** 初期キャリブレーションと GNSS データが利用可能で、**When** ロボットが巡回してセンサデータ
   （レーザ/レーザスキャン、オドメトリ、GNSS）を収集し、**Then** slam_toolbox を拡張したパイプラインが
   GNSS 拘束をポーズグラフ最適化に組み込み、生成された 2D 地図が地球座標系と整合する（複数地点で登録誤差の中央値 < 1 m を目標）。
2. **Given** 既存の 2D 地図があり GNSS トラックが存在する場合、**When** GNSS 限定の後処理を行う、**Then** 地図が
   GNSS によって再最適化され、地球座標に整合した地図が出力されること。

### Edge Cases
- GNSS の可用性が局所的に低下（遮蔽）：その領域では GNSS 拘束を弱めるかスイッチオフし、局所的に odom/scan-matching に依存する。
- GNSS の精度が粗悪（マルチパス等）：外れ値検出と重み付けにより誤った拘束の影響を低減する。
- 広域で一様ではない地図歪み：地図を分割して局所ごとに最適化後、全体整合を図る戦略を用いる。

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: The system MUST produce a 2D occupancy/costmap (slam_toolbox-compatible) aligned to a projected
   Earth coordinate frame (e.g., UTM) by incorporating GNSS observations as constraints in the SLAM pose graph.
- **FR-002**: GNSS observations MUST be preprocessed to convert lat/lon to the chosen projected frame and associated
   with SLAM poses via time synchronization (±100 ms) or interpolation.
- **FR-003**: The pose graph optimization MUST support per-constraint weights and robust loss functions to mitigate
   GNSS outliers (e.g., Huber, Cauchy) and allow region-specific weighting strategies.
- **FR-004**: The pipeline MUST provide an offline mode (post-processing) that takes an existing map + GNSS track and
   outputs a re-optimized, Earth-aligned map.
- **FR-005**: After map alignment, runtime localization MUST be achievable using `amcl` (or equivalent) with
   transforms provided by `robot_localization` to fuse GNSS/IMU/odometry for continuous pose estimates in Earth coordinates.
- **FR-006**: The system MUST export: aligned map file (PGM + YAML with Earth-frame origin), a pose graph dump, and
   a report summarizing GNSS residuals and final map-to-earth registration error statistics.

- **FR-007**: PoseGraph のシリアライズ / デシリアライズ機能は C++ で実装され、可能な限り `slam_toolbox` の既存ライブラリ/API を流用して行うこと。
   - 目的: ランタイムでの互換性と性能を確保し、既存の slam_toolbox の内部表現との整合を保つ。
   - 実装配置: 実装は `horiokart_slam` パッケージ内に配置すること（`app/horiokart_slam`）。

- **FR-008**: GNSS を含めたポーズグラフの最適化 PoC はまず Python で実装し（小さなスクリプト/Notebook 可）、動作確認と評価が済み次第、最終実装を C++ に移植すること。
   - PoC 言語: Python 3.11+（推奨）
   - 移行先: C++17/C++20 のコードベース（最終的にライブラリ化して CLI として利用可能にする）

*Non-functional Requirements*
- **NFR-001**: Target registration accuracy: median error < 1 m across open areas (RTK GNSS 前提、デプロイ毎に調整可能)。
- **NFR-002**: The post-processing pipeline SHOULD handle datasets of at least 2 km path length within reasonable time
   (e.g., < 30 min on a workstation class machine for typical dataset sizes) — profile and document performance.
- **NFR-003**: Logs and outputs MUST be reproducible; all deterministic random seeds (if any) MUST be recorded.

### Key Entities
- **PoseGraph**: Nodes = robot poses (time-stamped), Edges = scan-matching constraints, odometry, and GNSS constraints (with
   weight, covariance). Does NOT include implementation details of storage format.
- **GNSSObservation**: Time, lat, lon, alt, fix_type, reported_hdop/vdop, and associated covariance estimate.
- **AlignedMap**: 2D occupancy grid plus metadata declaring Earth-frame origin, projection (e.g., UTM zone), and map resolution.

## Detailed Definitions

以下は実装・テストのために明確化された定義。各項目はテスト可能な形式で記述する。

1. GNSS 入力仕様
   - topic: `/fix` (sensor_msgs/NavSatFix)
   - min_rate: 1 Hz
   - time_reference: `header.stamp` を ROS 時刻で利用
   - 必須フィールド: latitude, longitude, altitude, position_covariance (もし利用可能なら)

2. GNSS 前処理
   - datum: WGS84
   - 投影: UTM, zone は観測の中央値で自動選択
   - 精度フィルタ: hdop/推定共分散に基づき weight を設定（初期閾値: hdop < 2.0 を高信頼)

3. 時刻同期 / 補間
   - 許容ずれ: ±100 ms（目標）
   - 補間方法: 線形補間（位置）および共分散の簡易増幅

4. PoseGraph スキーマ
   - nodes: [{id:int, t:ISO8601, pose:[x,y,theta], cov:[9 numbers optional]}]
   - edges: [{from:int, to:int, type:'odom'|'scan'|'gnss', meas:[dx,dy,dtheta], info:[9 numbers optional]}]
   - ファイル形式: JSON (contracts/posegraph.schema.json に準拠)

5. GNSS→拘束変換
   - GNSS は unary 2D 位置拘束としてノードに付与（theta は無い）
   - 共分散: GNSS の水平共分散を使用し、座標系は UTM に変換
   - ロバスト化: Huber loss (delta=1.0) を初期設定

6. 評価指標
   - 主指標: median(L2(map_node_position, GNSS_reference)) < 1.0 m
   - 補助指標: mean, RMSE, 90th percentile
   - 最低サンプル数: N >= 20 location samples

7. 出力アーティファクト
   - aligned_map/: map.pgm, map.yaml (origin_utm), posegraph_aligned.json, registration_report.json (median, mean, RMSE, histogram)


## Implementation Notes (non-normative)

- Serialization implementation: implement a C++ component (library + small CLI) that uses `slam_toolbox` data structures or export helpers where available to serialize the internal pose graph to the agreed JSON schema (`contracts/posegraph.schema.json`) and to re-import it after optimization. This component MUST be implemented without changing the core behavior of `slam_toolbox` and should prefer using public APIs or minimal adapter code.

- Implementation strategy: Deliver Approach A first (service-based, non-invasive PoC). After validation, provide an optional Approach B (linked adapter) and design the codebase with a clear adapter interface so B can cleanly replace A. Both adapters must produce identical JSON schema outputs.


- Optimization PoC: develop an offline Python PoC that:
   1. Reads the serialized posegraph JSON and GNSS CSV/JSON (per `contracts/gnss.schema.json`).
   2. Converts GNSS observations to UTM and associates them with pose nodes (time sync/interpolation).
   3. Runs a pose-graph optimization using a Python optimizer (e.g., scipy least-squares or GTSAM Python bindings if available) with robust loss.
   4. Emits an optimized posegraph JSON compatible with the C++ deserializer for map regeneration.

- Migration: once PoC is validated, port the optimization logic to C++ (re-using the same data models) and provide a single C++ CLI (or library API) for production use.
   - 実装配置: PoC と最終実装は `horiokart_slam` パッケージへ統合する（`app/horiokart_slam/tools` 等に配置）。

## Implementation Policy (project-wide)

- Priority: algorithmic robustness over interactive usability. For the initial and production-focused implementations, prefer proven robust estimation methods (covariance-aware information weighting, IRLS/DCS for outlier handling, lever-arm/yaw joint estimation) and conservative defaults rather than polishing CLI ergonomics or UX features. Usability improvements (rich CLI, GUIs) are secondary and can be added after algorithms and tests are stable.
- GNSS covariance: assume the provided GNSS input includes horizontal covariance information (either as `position_covariance` in `NavSatFix` or derived from HDOP/VDOP fields). All GNSS-processing code MUST consume and propagate covariance information through interpolation/matching into the `constraints[].cov` field in the posegraph JSON.
- KartoAdapter requirement: the C++ adapter(s) MUST export all available data from the posegraph and underlying sensors when possible. At minimum the JSON exporter should include:
   - nodes[]: full timestamp, unique id/state_id, 2D pose (x,y,theta), node-level covariance (if available), any per-node metadata
   - edges[]: from/to indices or ids, measurement transform (dx,dy,dtheta), edge covariance OR information matrix when available, edge type label (`scan`|`odom`|`loop`), measurement source metadata (e.g., sensor id)
   - global metadata: map frame, projection (UTM zone), exporter version, timestamp of export

These exporter guarantees are required because downstream optimization and robustification depend on having accurate covariance and provenance information.

## Build & Test (developer instructions)

- Workspace: All C++ builds and ROS tests for this feature MUST be executed in the ROS2 workspace root: `/root/ros2_ws`.
- Install package dependencies using rosdep before building:

   ```bash
   cd /root/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

- When building or testing, always specify the package to limit scope. Example (build & test only `horiokart_slam`):

   ```bash
   cd /root/ros2_ws
   colcon build --packages-select horiokart_slam
   colcon test --packages-select horiokart_slam
   ```

- Rationale: limiting package scope (via `--packages-select`) reduces CI and developer iteration time and avoids accidental rebuilds of unrelated packages.



---

## Review & Acceptance Checklist
*GATE: Automated checks run during main() execution*

### Content Quality
- [ ] No low-level implementation details that preclude alternatives (keep tech notes in design)
- [ ] Focused on user value and why GNSS constraints are required
- [ ] All mandatory sections completed

### Requirement Completeness
- [ ] No [NEEDS CLARIFICATION] markers remain
- [ ] Requirements are testable and unambiguous
- [ ] Success criteria are measurable (registration error stats provided)
- [ ] Performance and scale expectations documented

## Execution Status
*Updated by main() during processing*

- [ ] User description parsed
- [ ] Key concepts extracted
- [ ] Ambiguities marked
- [ ] User scenarios defined
- [ ] Requirements generated
- [ ] Entities identified
- [ ] Review checklist passed

## Current Implementation Status (repository sync)

- Tasks and the implementation plan for this feature have been committed under `specs/001-slam-toolbox-gnss/`.
- The task list (`tasks.md`) has been created/updated and includes an ordered set of implementation iterations (T001..T008). See `tasks.md` for per-task details and current statuses (T001: in-progress).
- A PoC Python optimizer and a C++ Karto adapter prototype exist in the `horiokart_slam` package in the workspace; JSON export of posegraphs and timestamps has been validated on representative datasets. These artifacts should be consulted when implementing the next iterations (gnss_extract/transform/match/optimize).
