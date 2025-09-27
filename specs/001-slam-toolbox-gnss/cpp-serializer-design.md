# C++ Serializer Design

目的: `slam_toolbox` の内部ポーズグラフ表現を利用して、PoseGraph を JSON 形式でシリアライズ/デシリアライズする C++ ライブラリと CLI を提供する。

1) 要求 (高位)
- 入出力: `contracts/posegraph.schema.json` 準拠の JSON を読み書きできること。
- 既存の `slam_toolbox` 構造体/クラスを可能な限り再利用すること（改造は最小限）。
- CLI: `slam_posegraph_serialize --out posegraph.json` / `slam_posegraph_deserialize --in posegraph.json` を提供。

2) 基本 API（ヘッダ草案）

```cpp
// posegraph_serializer.h
namespace slam_gnss {
  struct Node { int id; double t; double x,y,theta; std::array<double,9> cov; };
  struct Edge { int from; int to; std::string type; std::array<double,3> meas; std::array<double,9> info; };

  class PoseGraphSerializer {
  public:
    // Serialize in-memory posegraph (from slam_toolbox structures) to JSON file
    static bool Serialize(const SlamToolboxGraph& graph, const std::string &out_path);

    // Deserialize JSON file into slam_toolbox compatible structure
    static bool Deserialize(const std::string &in_path, SlamToolboxGraph &out_graph);
  };
}
```

注: `SlamToolboxGraph` は既存の slam_toolbox 内のグラフ表現に合わせる（存在しない場合は minimal adapter を実装）。

3) 依存とビルド
- 依存: ROS2 (rclcpp), slam_toolbox 開発ヘッダ、nlohmann/json（軽量 JSON）、Eigen（行列操作）、CMake ビルド。
- C++ 標準: C++17 以上推奨。

4) テスト方針
- ユニット: 小さなグラフをメモリで生成→Serialize→Deserialize→一致性チェック。
- 統合: 実機/シミュレーションで `slam_toolbox` が稼働中に Serialize を実行し、Deserialize 後に map を再生成可能であること。
- CI: コンテナでビルド & 単体テストを実行するジョブを追加。

5) CLI 仕様
- `slam_posegraph_serialize --out posegraph.json --topic /slam_toolbox/graph_visualization`（既存トピック監視でダンプ）
- `slam_posegraph_deserialize --in posegraph.json --apply-map --out-dir ./aligned_map`（ディスクに最終地図を生成するオプション）

6) 安全性と互換性
- 既存の slam_toolbox 振る舞いを変更しない（読み取り専用/adapter パターンを採用）。
- バージョニング: 出力 JSON に `schema_version` フィールドを含め、将来の互換性を確保。

7) 実装ステップ（推奨）
 - Step 1: slam_toolbox のグラフ型と吐き出しトピックを調査し、最小 adapter の API を決定する。
 - Step 2: Serialize/Deserialize のプロトタイプを作成（nlohmann/json 使用）。
 - Step 3: ユニットテスト作成と CI ワークフローに組込む。
 - Step 4: CLI と integration test（rosbag で end-to-end の smoke test）を追加。
