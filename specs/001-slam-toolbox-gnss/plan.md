
# Implementation Plan: [FEATURE]

**Branch**: `[###-feature-name]` | **Date**: [DATE] | **Spec**: [link]
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

## Execution Flow (/plan command scope)
```
1. Load feature spec from Input path
   → If not found: ERROR "No feature spec at {path}"
2. Fill Technical Context (scan for NEEDS CLARIFICATION)
   → Detect Project Type from context (web=frontend+backend, mobile=app+api)
   → Set Structure Decision based on project type
3. Fill the Constitution Check section based on the content of the constitution document.
4. Evaluate Constitution Check section below
   → If violations exist: Document in Complexity Tracking
   → If no justification possible: ERROR "Simplify approach first"
   → Update Progress Tracking: Initial Constitution Check
5. Execute Phase 0 → research.md
   → If NEEDS CLARIFICATION remain: ERROR "Resolve unknowns"
6. Execute Phase 1 → contracts, data-model.md, quickstart.md, agent-specific template file (e.g., `CLAUDE.md` for Claude Code, `.github/copilot-instructions.md` for GitHub Copilot, `GEMINI.md` for Gemini CLI, `QWEN.md` for Qwen Code or `AGENTS.md` for opencode).
7. Re-evaluate Constitution Check section
   → If new violations: Refactor design, return to Phase 1
   → Update Progress Tracking: Post-Design Constitution Check
8. Plan Phase 2 → Describe task generation approach (DO NOT create tasks.md)
9. STOP - Ready for /tasks command
```

**IMPORTANT**: The /plan command STOPS at step 7. Phases 2-4 are executed by other commands:
- Phase 2: /tasks command creates tasks.md
- Phase 3-4: Implementation execution (manual or via tools)

## Summary
[Extract from feature spec: primary requirement + technical approach from research]

## Technical Context
**Language/Version**: [e.g., Python 3.11, Swift 5.9, Rust 1.75 or NEEDS CLARIFICATION]  
**Primary Dependencies**: [e.g., FastAPI, UIKit, LLVM or NEEDS CLARIFICATION]  
**Storage**: [if applicable, e.g., PostgreSQL, CoreData, files or N/A]  
**Testing**: [e.g., pytest, XCTest, cargo test or NEEDS CLARIFICATION]  
**Target Platform**: [e.g., Linux server, iOS 15+, WASM or NEEDS CLARIFICATION]
**Project Type**: [single/web/mobile - determines source structure]  
**Performance Goals**: [domain-specific, e.g., 1000 req/s, 10k lines/sec, 60 fps or NEEDS CLARIFICATION]  
**Constraints**: [domain-specific, e.g., <200ms p95, <100MB memory, offline-capable or NEEDS CLARIFICATION]  
**Implementation Prioritization**: Favor algorithmic robustness and correctness over usability polish for the initial and production-focused iterations. This means: prioritize implementing covariance-aware weighting, robust loss and outlier-rejection (IRLS/DCS), and lever-arm estimation before investing in CLI ergonomics or GUIs. Document the rationale and conservative defaults in `quickstart.md`.
**Scale/Scope**: [domain-specific, e.g., 10k users, 1M LOC, 50 screens or NEEDS CLARIFICATION]

## Constitution Check
*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Gates determined based on constitution file]

### Quick Constitution Check (automated)

- Library-First: OK — this feature describes offline CLI-style tooling and data models; implementation should start as a standalone library/tool (no core runtime changes to `slam_toolbox`).
- CLI/Text-First: OK — quickstart and contracts are text-first; outputs are JSON/YAML per contract.
- Test-First: PARTIAL — contracts and schemas were generated; contract tests are not yet created and must be added in Phase 1 outputs.
- Integration/Contract Testing: PARTIAL — contracts produced (schemas) satisfy the requirement to define contracts; implementers must add contract tests before merge.
- Observability/Versioning & Simplicity: OK — quickstart and research recommend reproducible run manifests; further docs to include logging/metrics guidance.

Complexity Notes:
- We are not modifying `slam_toolbox` (meets Library-First). If a wrapper node is required to export posegraph at runtime, keep it minimal and library-wrapped.


## Project Structure

### Documentation (this feature)
```
specs/[###-feature]/
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)

### KartoAdapter (export requirements)

Implementers MUST ensure the C++ KartoAdapter (or service-based adapter) exports a complete data dump sufficient for offline covariance-aware optimization. At minimum the adapter should include:
- nodes[]: id, state_id, timestamp, pose [x,y,theta], node-level covariance (if available), sensor/source metadata
- edges[]: from/to (both id and index if possible), measured transform [dx,dy,dtheta], covariance or information matrix, edge type label, and source metadata
- metadata: exporter tool/version, map frame, projection info (UTM zone), and generation timestamp

This explicit contract reduces ambiguity for the Python PoC and later C++ optimizer and is a hard requirement for T002 (C++ PoseGraph prototype stabilization).
```

### Source Code (repository root)
```
# Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure]
```

**Structure Decision**: [DEFAULT to Option 1 unless Technical Context indicates web/mobile app]

## Phase 0: Outline & Research
1. **Extract unknowns from Technical Context** above:
   - For each NEEDS CLARIFICATION → research task
   - For each dependency → best practices task
   - For each integration → patterns task

2. **Generate and dispatch research agents**:
   ```
   For each unknown in Technical Context:
     Task: "Research {unknown} for {feature context}"
   For each technology choice:
     Task: "Find best practices for {tech} in {domain}"
   ```

3. **Consolidate findings** in `research.md` using format:
   - Decision: [what was chosen]
   - Rationale: [why chosen]
   - Alternatives considered: [what else evaluated]

**Output**: research.md with all NEEDS CLARIFICATION resolved

## Phase 1: Design & Contracts
*Prerequisites: research.md complete*

1. **Extract entities from feature spec** → `data-model.md`:
   - Entity name, fields, relationships
   - Validation rules from requirements
   - State transitions if applicable

2. **Generate API contracts** from functional requirements:
   - For each user action → endpoint
   - Use standard REST/GraphQL patterns
   - Output OpenAPI/GraphQL schema to `/contracts/`

3. **Generate contract tests** from contracts:
   - One test file per endpoint
   - Assert request/response schemas
   - Tests must fail (no implementation yet)

4. **Extract test scenarios** from user stories:
   - Each story → integration test scenario
   - Quickstart test = story validation steps

5. **Update agent file incrementally** (O(1) operation):
   - Run `.specify/scripts/bash/update-agent-context.sh copilot`
     **IMPORTANT**: Execute it exactly as specified above. Do not add or remove any arguments.
   - If exists: Add only NEW tech from current plan
   - Preserve manual additions between markers
   - Update recent changes (keep last 3)
   - Keep under 150 lines for token efficiency
   - Output to repository root

**Output**: data-model.md, /contracts/*, failing tests, quickstart.md, agent-specific file

### Additional Phase 1 deliverables for implementation language plan

- C++ Serializer component: Design document and contract for a C++ component that serializes/deserializes PoseGraph using `slam_toolbox`'s native structures where available. This component shall produce/consume `posegraph.schema.json`.
- Python PoC design: a minimal Python implementation plan (not production) that reads the serialized posegraph, applies GNSS constraints, runs optimization, and writes optimized posegraph JSON.
 - C++ Serializer design doc: `/app/specs/001-slam-toolbox-gnss/cpp-serializer-design.md`

## Phase 2: Task Planning Approach
*This section describes what the /tasks command will do - DO NOT execute during /plan*

**Task Generation Strategy**:
- Load `.specify/templates/tasks-template.md` as base
- Generate tasks from Phase 1 design docs (contracts, data model, quickstart)
- Each contract → contract test task [P]
- Each entity → model creation task [P] 
- Each user story → integration test task
- Implementation tasks to make tests pass

## Language-Transition Plan (PoC -> Production)

- Step A (Phase 1): Implement C++ serializer/deserializer. This is a prerequisite for an end-to-end test that reimports optimized graphs into C++ for map regeneration.
- Step B (Phase 2): Implement Python PoC optimizer (small, iterative, testable). Deliverables: scripts/notebooks, sample data, evaluation report.
- Step C (Phase 3): Port Python optimizer to C++ (use same schema). Deliverables: C++ optimizer library + CLI, integration tests, performance benchmarks.


**Ordering Strategy**:
- TDD order: Tests before implementation 
- Dependency order: Models before services before UI
- Mark [P] for parallel execution (independent files)

**Estimated Output**: 25-30 numbered, ordered tasks in tasks.md

**IMPORTANT**: This phase is executed by the /tasks command, NOT by /plan

## Phase 3+: Future Implementation
*These phases are beyond the scope of the /plan command*

**Phase 3**: Task execution (/tasks command creates tasks.md)  
**Phase 4**: Implementation (execute tasks.md following constitutional principles)  
**Phase 5**: Validation (run tests, execute quickstart.md, performance validation)

## Complexity Tracking
*Fill ONLY if Constitution Check has violations that must be justified*

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |


## Progress Tracking
*This checklist is updated during execution flow*

**Phase Status**:
Phase 0: research.md — Completed
Phase 1: data-model.md, contracts/, quickstart.md — Completed (schemas present; contract tests pending)

**Gate Status**:
- Constitution Check: Passed with PARTIAL items (contract tests missing)

## Current Execution State

- The implementation plan (this file) and the feature spec (`spec.md`) are in sync.
- The task list `tasks.md` was generated and saved. Primary work has started on T001 (contracts validation tests) and prototype artifacts for `karto_adapter` and a Python PoC optimizer exist in the workspace (`horiokart_slam`).
- Next recommended step (per plan): implement `tools/gnss_extract.py` (T003) after T001 reaches a stable test harness.

## Workspace Implementation Snapshot

Based on the current repository under `src/horiokart/horiokart_slam/`, the following artifacts are present and should be considered part of the implementation baseline:

- C++ adapter & serializer prototypes:
   - `src/posegraph_serializer/karto_adapter.cpp`, `karto_adapter_test.cpp`, `service_adapter.cpp` — code to read Karto `.data`/.posegraph and export JSON.
   - `include/horiokart_slam/karto_adapter.hpp`, `posegraph_adapter.hpp`, `service_adapter.hpp` — adapter interfaces and declarations.
- Python PoC & tools:
   - `tools/poc_optimize.py` — Python PoC optimizer.
   - `tools/gnss_match.py`, `tools/gnss_optimize.py`, `tools/gnss_synth.py` — existing helper scripts related to matching/optimization/synthesis.
- Integration scripts & launchers:
   - `scripts/run_gnss_fusion.py` — an integration runner script exists (needs wiring/parameterization).
   - ROS launch files `launch/bringup_slam_toolbox.launch.py`, `launch/record_bag.launch.py` are present to exercise slam_toolbox.

Implication for plan phases:
- Several Phase 1/Phase 2 artifacts already exist as prototypes; Task statuses should reflect that T002 (C++ serializer) has an implementation in-progress and T005/T006 (matching & optimization) have PoC code present. T003/T004 (gnss extraction & transform) remain to be implemented.

*Based on Constitution v2.2.0 - See `/memory/constitution.md`
