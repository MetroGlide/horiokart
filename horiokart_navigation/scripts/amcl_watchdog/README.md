# AMCL Watchdog

この小さなパッケージは AMCL の推定精度（位置共分散）を監視し、指定回数連続してしきい値を超えた場合に initialpose を再設定する機能を提供します。

主な機能
- `/amcl_pose` を購読して共分散から監視指標（デフォルト: trace_xy）を計算
- 連続 N 回のしきい値超過で異常と判定し、再初期化ハンドラを呼び出す
- デフォルトでは `gnss_amcl_initializer`（サービス呼び出し）を試行します（フォールバック publish は未実装）

使い方（概要）
1. ワークスペースをビルドして source する
2. launch ファイル `horiokart_navigation/launch/localization_launch.py` を使って起動すると、amcl_watchdog ノードがタイマーで起動されます（デフォルト設定）

主なパラメータ（ノード名: `amcl_watchdog`）
- `metric` : `trace_xy` | `determinant_xy` | `max_eigenvalue_xy`（デフォルト `trace_xy`）
- `threshold` : float（デフォルト `2.0`、metric に対応する単位）
- `consecutive_count` : int（デフォルト `3`）
- `initializer.type` : `service` | `topic`（デフォルト `service`）
- `initializer.service_name` : string（デフォルト `/gnss_amcl_initializer_node/request_reinit`）
- `recovery_backoff_sec` : float（デフォルト `60.0`）
- `max_retries` : int（デフォルト `3`）

注意
- 実際の `gnss_amcl_initializer` のサービス名/型が異なる場合は、launch で `initializer.service_name` を合わせるか、ハンドラを調整してください。

開発者向け
- 実装コードは `horiokart_navigation/amcl_watchdog/` パッケージ内にあります: `metrics.py`, `detectors.py`, `handlers.py`, `types.py` など。`scripts/amcl_watchdog/amcl_watchdog_node.py` はラッパースクリプトです。
- 異常判定ロジックや再設定ハンドラは簡単に差し替え可能です。

処理の流れ（ASCII アート）

以下はノードの主要な処理フローを図示したものです。簡潔に入力→判定→再設定の流れを表しています。

```
		    +----------------------+
		    |  /amcl_pose トピック  |
		    | PoseWithCovarianceStamped |
		    +----------+-----------+
				   |
				   v
			+--------------------+
			| metrics.compute()  |  <-- 共分散から指標を計算
			| (trace_xy 等)      |
			+---------+----------+
				    |
				    v
			+--------------------+
			| AnomalyDetector    |  <-- 連続閾値判定 (ConsecutiveThreshold)
			| (feed(value))      |
			+----+----------+----+
			     |          |
		    正常 |          | 異常 (連続N回)
			     |          v
			     |   +--------------+
			     |   | AnomalyEvent  |
			     |   +------+-------+
			     |          |
			     |          v
			     |   +--------------+
			     |   | RecoveryHandler| <-- 抽象インターフェース
			     |   +----+---------+
			     |        |
			     |        |---> (service 呼び出し) gnss_amcl_initializer
			     |        |         例: std_srvs/Trigger へリクエスト
			     |        |
			     |        |---> (fallback) /initialpose へ Publish
			     |                  PoseWithCovarianceStamped を送信
			     v
		     (ログ/診断出力)
			     |
			     v
		   +-------------------------------+
		   | AmclWatchdogNode              | <-- backoff, retry制御, フラッピング抑止, ログ/警告/監視
		   +-------------------------------+
- `gnss_amcl_initializer` のサービス仕様に合わせて `RecoveryHandler` を調整してください。

