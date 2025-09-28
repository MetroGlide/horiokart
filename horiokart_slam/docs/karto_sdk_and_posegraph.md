# Karto SDK と posegraph/.data ファイル形式（再利用ドキュメント）

目的
- `karto_sdk` を利用して `slam_toolbox` が保存する posegraph（.posegraph）および dataset（.data）ファイルを解析・再利用するための仕様と実装メモをまとめる。
- 今後の adapter 実装やオフライン解析（GNSS と合わせた地球座標系への変換）で再利用できる知見を提示する。

前提
- 本リポジトリでは `slam_toolbox` が内部的に `karto_sdk` を用いて `Mapper`（pose graph）と `Dataset`（センサデータ／スキャン）を Boost.Serialization 形式で保存する。
- ファイルはデフォルトで二つのファイルセットとして保存される（例）:
  - `myrun.posegraph.posegraph`  -> `karto::Mapper` を Boost バイナリアーカイブで保存したファイル
  - `myrun.posegraph.data`      -> `karto::Dataset` を Boost バイナリアーカイブで保存したファイル

概要（高レベル）
- `Mapper`:
  - ノード（頂点）とエッジ（制約）を表すグラフ構造を保持する。
  - ノードはロボットの時刻順に割り当てられた一意の `state_id`（内部ID）を持つことがある。
  - ノードの位置（pose）、およびエッジ（相対pose）とその共分散行列（3x3）を持つ。
- `Dataset`:
  - センサデータ（LaserRangeScan 等）の集合を保持する。
  - 各 `SensorData` は時間（m_Time）を持ち、これは Karto の `SensorData::GetTime()` / `SetTime()` で参照される。
  - `LocalizedRangeScan`（位置付きスキャン）は `LaserRangeScan` -> `SensorData` を継承しており、スキャン点とローカライズ結果、状態ID などを含む。

ファイルフォーマット詳細
- 保持形式: Boost.Serialization のバイナリアーカイブ（`boost::archive::binary_oarchive`/`binary_iarchive`）
- `*.posegraph.posegraph`:
  - `karto::Mapper` を Boost でシリアライズしたもの。
  - 内容（注: 実装依存・将来的に変わる可能性あり）:
    - Mapper の内部オブジェクト（Graph、Vertices、Edges、ParameterManager 等）
    - 各頂点（vertex）に対応する位置（x,y,theta）情報、および対応付けられる `state_id`（実装により存在しない場合あり）
- `*.posegraph.data`:
  - `karto::Dataset` を Boost でシリアライズしたもの。
  - 内容:
    - SensorData（LaserRangeScan、LocalizedRangeScan 等）の配列/マップ
    - 各 `SensorData` は `m_Time`（kt_double）フィールドを持つ。Boost シリアライズ定義に含まれるため、ファイルに保存される。
    - 追加情報として、スキャンのレンジデータ、（必要に応じて）ポイントクラウド、状態ID（state id）への参照などが含まれることがある。

Karto の主な型メモ（実装参照）
- karto::SensorData
  - m_Time: kt_double（GetTime()/SetTime() を通じてアクセス）
  - 目的: センサデータのタイムスタンプ（Karto 内部で経過時刻や epoch 秒で扱われることがある）
- karto::LaserRangeScan
  - Laser のレンジ読み取りを表す
  - 継承元: SensorData
- karto::LocalizedRangeScan
  - 位置情報（ローカライズ済みのスキャン）、および state id などを含む
  - 継承: LaserRangeScan（→ SensorData）
- karto::Dataset
  - SensorData の集合を保持
  - serialize(ar, version) が実装されており Boost でデータを保存/復元する
- karto::Mapper
  - Graph（Vertices/Edges）を保持
  - Mapper も Boost シリアライズを実装

注意: 時間（timestamp）の意味と基準
- `SensorData::m_Time` の中身（基準）は利用状況に依存します。
  - slam_toolbox の実装では、受信した ROS のスキャンメッセージの `header.stamp` を Karto の `range_scan->SetTime(...)` にセットしてから Mapper/Dataset を作成しているため、通常は ROS 時刻（ROS 時刻基準、通常はシステム時刻か /use_sim_time の設定に依存）となる。
  - 実際の値が 0 になるケースは、slam_toolbox 実行時に `header.stamp` が 0 であったか、もしくは相対時刻で保存された場合に起こる。
- 照合を行う前に、GNSS 側の時刻と Karto 側（SensorData::m_Time）の基準・単位が一致するか確認してください。
  - 必要ならばオフセット（epoch の差）やスケール（秒 vs ナノ秒）を補正すること。

取り出し方・再利用パターン
- 1) slam_toolbox が走っている ROS2 ノードに対しシリアライズサービスを呼ぶ（手軽）
  - サービス: `/slam_toolbox/serialize_map` (srv: `slam_toolbox/srv/SerializePoseGraph`)
  - request: string filename（拡張子は付けない。内部で filename + ".posegraph" / ".data" として保存される）
  - 例:
    - ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/root/ros2_data/slam_modify/automatic_test.posegraph'}"
  - 出力: `/root/ros2_data/slam_modify/automatic_test.posegraph.posegraph` と `.data`
- 2) Karto SDK を使って .posegraph/.data を読み込む
  - C++ 例（本リポジトリ内の `KartoAdapter` 実装参照）:
    - karto::Mapper mapper; mapper.LoadFromFile(posegraph_path);
    - karto::Dataset dataset; dataset.LoadFromFile(dataset_path);
    - dataset 内の LocalizedRangeScan から `GetTime()` と `GetStateId()` を取り出す
- 3) JSON 等で中間形式にエクスポート
  - 本プロジェクトでは nlohmann_json を用いて `nodes` と `edges` を含む JSON 形式に変換する実装を提供済み
  - nodes に `id`, `pose`, `state_id`, `timestamp` を含めることで、外部の最適化器（Python PoC optimizer 等）で GNSS とマッチングしやすくする

留意点（実運用でのチェックリスト）
- 出力された `.data` に timestamp が含まれているかを目視・プログラムで確認する（今回のテストで非ゼロが確認できた）
- GNSS と比較する場合、GPS の time stamp（通常は POSIX epoch 秒）との比較を行うため、Karto 側の `m_Time` がどの epoch/単位かを必ず確認する（必要なら補正する）
- Boost.Serialization のバイナリ形式はバージョンやコンパイラ・ABI に依存する可能性があるため、別のマシンで読み書きする際は互換性に注意する

実装スニペット（参照）
- `ros2 service call` でシリアライズ

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/root/ros2_data/slam_modify/automatic_test.posegraph'}"
```

- C++ での読み込み（擬似）：

```cpp
karto::Mapper mapper;
mapper.LoadFromFile(posegraph_file);

karto::Dataset dataset;
dataset.LoadFromFile(data_file);

for (auto & scan_pair : dataset.GetScans()) {
  auto scan = dynamic_cast<karto::LocalizedRangeScan *>(scan_pair.second.get());
  if (scan) {
    double t = scan->GetTime();
    int state_id = scan->GetStateId();
    // pose, ranges 等を取り出して JSON に格納する
  }
}
```

付録: 本ドキュメントに関連するリポジトリ内ファイル
- `src/posegraph_serializer/karto_adapter.cpp` — Karto から読み取り JSON に変換する実装（このドキュメントに対応）
- `src/posegraph_serializer/karto_adapter_test.cpp` — テストバイナリ。デフォルトで `/root/ros2_data/slam_modify/YYYYmmdd_HHMMSS/posegraph_from_karto.json` を生成
- `tools/poc_optimize.py` — JSON を受け取り GNSS と合わせて最適化する PoC スクリプト

---

作業メモ
- このドキュメントは実装に基づいて作成しており、将来的に Karto SDK の変更や slam_toolbox の振る舞い変更があれば更新が必要です。
- 互換性や epoch の扱いは重要なポイントなので、運用上の検証（いくつかのログで timestamp の比較確認）をおすすめします。
