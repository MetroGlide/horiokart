````markdown
# tasks.md

以下はこのフィーチャーの実行可能なタスクリストです。各タスクには依存関係を示し、優先度と並列化可否を記載しています。

## 実行ルール
- タスク ID は T### 形式
- 先行タスクがある場合は Dependencies に TID を列挙
- [P] が付いているタスクは他の独立タスクと並列で進められる

---

## T001 — 契約テスト作成 (in-progress)
- Title: contracts validation tests
- Description: contracts/*.schema.json に従って JSON を検証する pytest ベースのテストハーネスを作成。小さな合成フィクスチャ(posegraph, gnss) を用意し、テスト駆動で仕様を固定する。
- Files: specs/001-slam-toolbox-gnss/tests/test_contracts.py, specs/001-slam-toolbox-gnss/tests/fixtures/posegraph_small.json, specs/001-slam-toolbox-gnss/tests/fixtures/gnss_small.json
- Dependencies: none
- Estimated: 1 day
- Parallelizable: no

## T002 — C++ シリアライザ プロトタイプ
- Title: PoseGraph C++ Serialize/Deserialize prototype
- Description: slam_toolbox の公開 API または adapter を用いて、PoseGraph の Serialize/Deserialize を実装。ユニットテストを含む。
- Files: src/posegraph_serializer/*, tests/unit/serializer_test.cpp
 - Note: 実装は `horiokart_slam` パッケージに追加すること（`app/horiokart_slam` 内に配置）。
- Dependencies: T001 (contracts must be stable)
- Estimated: 3 days
- Parallelizable: partially [P]

---
Notes / Implementation hints:
- slam_toolbox にはすでに `SerializePoseGraph.srv` / `DeserializePoseGraph.srv` が定義されており、サービス経由でファイル出力(.posegraph/.data) が可能です。加えて `include/slam_toolbox/serialization.hpp` にファイル読み書き用ヘルパーが用意されています。
- 実装アプローチ (いずれか):
- どちらの方法でも出力 JSON は `contracts/posegraph.schema.json` に準拠すること。
- 実装時に確認する点: サービス名（ノードによる登録名）、ファイル保存先のパス権限、karto::Mapper からノード/エッジを列挙する API の把握。

Selected implementation strategy for T002:
- First deliverable: use Approach A (Non-invasive) to create a fast PoC that calls `SerializePoseGraph` service, reads the produced `.posegraph`/`.data` files, and converts them to `contracts/posegraph.schema.json`-compliant JSON. Implement this PoC inside `horiokart_slam` (e.g., `app/horiokart_slam/src/posegraph_serializer/service_adapter`).
- Later migration: implement Approach B (Linked adapter) to extract graph data directly from `karto::Mapper`/`karto::Dataset` for performance and tighter integration. The B implementation will live alongside the A implementation and can replace it by switching an adapter implementation at build/run time.

Design constraints to enable easy swap:
- Define a small adapter interface (C++ abstract class) `IPoseGraphAdapter` with methods `LoadFromService(...)`, `LoadFromFiles(...)`, `ExportToJson(...)`.
- Implement two concrete adapters: `ServiceAdapter` (A) and `KartoAdapter` (B). The rest of the pipeline depends only on `IPoseGraphAdapter`.
- Keep JSON conversion logic and schema validation in a single module so both adapters reuse it.
- Add CMake options / build tags to control whether `KartoAdapter` (with Karto SDK link) is built; CI job can first build only `ServiceAdapter` for faster iteration.

## T003 — Python PoC オプティマイザ
- Title: GNSS-constrained optimizer PoC (Python)
- Description: JSON PoseGraph と GNSS を読み、UTM 変換、時刻同期、ロバスト損失で最適化する PoC を作成。評価レポートの出力。
- Files: tools/poc_optimize.py or notebooks/poc_optimize.ipynb, tests/integration/poc/
 - Note: PoC および最終コードは `horiokart_slam` パッケージへ統合する計画（`app/horiokart_slam/tools`）。
- Dependencies: T001
- Estimated: 2-3 days
- Parallelizable: yes [P]

## T004 — C++ へ最適化ロジック移植
- Title: Port optimizer to C++ (ceres/g2o)
- Description: PoC を基に C++ 実装を作成し、CLI を提供する。性能とメモリ確認を行う。
- Files: src/optimizer_cpp/*, bin/slam_gnss_optimize
- Dependencies: T002, T003
- Estimated: 5 days
- Parallelizable: no

## T005 — rosbag 統合テスト（E2E）
- Title: ROSBAG E2E smoke tests
- Description: サンプル rosbag を使って、PoseGraph の抽出→最適化→地図再生成を検証する。rosbag fixtures を CI に含める。
- Files: tests/e2e/test_rosbag_smoke.py, ci/rosbag_fixtures/
- Dependencies: T002, T004
- Estimated: 2 days
- Parallelizable: no

## T006 — CLI とクイックスタート整備
- Title: CLI docs and quickstart
- Description: quickstart.md を基にユーザー向けの手順を整備。例: slam_gnss_optimize のオプション説明と出力例。
- Files: docs/quickstart.md, README.md
- Dependencies: T003 (PoC usage) for examples; T004 for production CLI
- Estimated: 1 day
- Parallelizable: yes [P]

## T007 — CI ワークフロー追加
- Title: Add CI for contracts/unit/integration
- Description: .github/workflows/ci-slam-gnss.yml を追加し、contracts tests と unit tests を走らせる。
- Files: .github/workflows/ci-slam-gnss.yml
- Dependencies: T001, T002
- Estimated: 1 day
- Parallelizable: yes [P]

## T008 — 性能検証と NFR 確認
- Title: Performance profiling and NFR validation
- Description: ベンチ（2 km dataset）で NFR-002 を確認。プロファイル手順と結果を perf/profile_report.md にまとめる。
- Files: perf/profile_report.md
- Dependencies: T004
- Estimated: 2 days
- Parallelizable: no

## T009 — レビューとマージ準備
- Title: PR & acceptance checklist
- Description: PR を作成し、Acceptance Checklist を満たす確認を行う。必要なドキュメントと CI パスを確認。
- Files: PR description, acceptance/checklist
- Dependencies: all implementation tasks
- Estimated: 1 day

---

### Notes
- まずは T001 を完了させる。テスト駆動で contracts を固定化し、PoC と C++ 実装の基礎にする。
- T002 と T003 は並列で進められるが、C++ 実装は T002 の API 安定が望ましい。

````
