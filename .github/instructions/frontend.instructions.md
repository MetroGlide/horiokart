---
description: "Use when adding features, components, pages, or ROS topic/service integrations to the mg_web_ui frontend. Covers foxglove hooks, schema definitions, type patterns, and page structure."
applyTo: "mg_ui/mg_web_ui/frontend/src/**"
---

# mg_web_ui フロントエンド開発ガイド

技術スタック: React 18 + TypeScript + Vite + Tailwind CSS  
ROS通信: foxglove_bridge `ws://localhost:8765`  
SystemManager API: `http://<hostname>:8001`

## アーキテクチャ

`App.tsx` で `useFoxgloveClient()` と `useSystemManagerClient()` を一度だけ呼び出し、各ページに Props として渡す。Context は使わない。

```
App.tsx
├── client = useFoxgloveClient()      → FoxgloveClientHandle
├── sysManager = useSystemManagerClient()  → SystemManagerHandle
└── <SomePage client={client} sysManager={sysManager} />
```

### Context（使う場面）

| Context                | フック               | 用途                                            |
| ---------------------- | -------------------- | ----------------------------------------------- |
| `SimulationContext`    | `useSimulation()`    | `isSimulation: boolean` でシミュ/実機の表示切替 |
| `TeleopContext`        | `useTeleop()`        | テレオペ速度ゲージ設定                          |
| `VisualizationContext` | `useVisualization()` | 3Dビューアのレイヤー・オーバーレイ表示切替      |

## トピック購読

### 手順

1. `types/ros.ts` に受信データの型 `interface` を追加
2. `ros/topics.ts` にトピック名を追加
3. コンポーネント内で `useTopicSubscriber<T>` を呼ぶ

```ts
// 1. types/ros.ts
export interface MyMessage {
  value: number;
  label: string;
}

// 2. ros/topics.ts
export const TOPICS = {
  // ...既存...
  MY_TOPIC: nodeNs(NODE_NS.MY_NODE, "/my_topic"), // namespace あり
  MY_GLOBAL: "/some_global_topic", // グローバル
} as const;

// 3. コンポーネント
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { TOPICS } from "../ros/topics";
import type { MyMessage } from "../types";

const data = useTopicSubscriber<MyMessage>(
  client,
  TOPICS.MY_TOPIC,
  "pkg/msg/MyMessage",
);
```

**ポイント:**

- `useTopicSubscriber` の第3引数は `'pkg/msg/MsgType'` 形式の文字列（`schemas.ts` への登録は**不要**）
- 戻り値は `T | null`（未受信時は null）
- `client` は `FoxgloveClientHandle` を Props から受け取る

## ノード名前空間

```ts
// ros/namespaces.ts
export const NODE_NS = {
  WAYPOINT_SEQUENCER: "waypoint_sequencer_node",
  DIAGNOSTICS: "", // 空文字 = グローバル
} as const;

// 使い方: nodeNs(ns, path)
// nodeNs('waypoint_sequencer_node', '/status') → '/waypoint_sequencer_node/status'
// nodeNs('', '/diagnostics')                   → '/diagnostics'
```

新ノードを追加する場合は `NODE_NS` に定数を追加してから `nodeNs()` を使う。

## サービスコール

```ts
// 1. ros/services.ts
export const SERVICES = {
  MY_SERVICE: nodeNs(NODE_NS.MY_NODE, "/my_service"),
} as const;

// 2. コンポーネント
import { useServiceCaller } from "../hooks/useServiceCaller";
import { SERVICES } from "../ros/services";

const { call, loading, error } = useServiceCaller(client);

const handleClick = async () => {
  const result = await call(SERVICES.MY_SERVICE, { key: "value" });
};
```

- `call()` はタイムアウト 10000ms で `Promise<unknown>` を返す
- サービス定義ファイル: [ros/services.ts](../../mg_ui/mg_web_ui/frontend/src/ros/services.ts)

## トピックへのパブリッシュ

publish は foxglove経由でROSに送信するため `schemas.ts` への登録が必要。

```ts
// 1. ros/schemas.ts にスキーマを追加
export const SCHEMAS: Record<string, RosSchema> = {
  "pkg/msg/MyMessage": {
    encoding: "cdr",
    schemaName: "pkg/msg/MyMessage",
    // 依存する複合型は '===' 区切りで展開する
    schema: "int32 value\nstring label",
  },
};

// 2. publish 呼び出し
client.publish(TOPICS.MY_TOPIC, "pkg/msg/MyMessage", {
  value: 1,
  label: "hello",
});
```

- `schemas.ts` への登録は **publish 時のみ**必要（subscribe は不要）
- 複合型を含む場合は依存型を `===` 行区切りで全展開する（既存の `PauseRequest` を参照）

## ページコンポーネントの構造

ロボット操作系ページは `RobotPageLayout` を使い、左サイドバー（`SideAccordion`）と右3Dビューアを並べるレイアウトを構成する。

```tsx
// pages/MyPage.tsx
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import RobotPageLayout from "../components/RobotPageLayout";
import { AccordionItem } from "../components/SideAccordion";

export default function MyPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const accordionItems: AccordionItem[] = [
    {
      id: "status",
      label: "Status",
      children: <div>...</div>,
    },
  ];

  return (
    <RobotPageLayout
      client={client}
      accordionItems={accordionItems}
      defaultOpen={["status"]} // 初期展開するアコーディオン id
      viewerMode="2d" // '2d' | '3d' | undefined（undefined でビューア非表示）
    />
  );
}
```

新ページを追加する場合:

1. `pages/MyPage.tsx` を作成
2. `App.tsx` に `<Route path="/my-path" element={<MyPage client={client} sysManager={sysManager} />} />` を追加
3. `components/NavBar.tsx` にナビリンクを追加

### RobotPageLayout の Props

| Prop             | 型                          | 説明                                             |
| ---------------- | --------------------------- | ------------------------------------------------ |
| `client`         | `FoxgloveClientHandle`      | foxgloveクライアント                             |
| `accordionItems` | `AccordionItem[]`           | サイドバーのアコーディオン項目                   |
| `defaultOpen`    | `string[]`                  | 初期展開する項目の id リスト                     |
| `viewerMode`     | `'2d' \| '3d' \| undefined` | 3Dビューアのモード。`undefined` でビューア非表示 |
| `extraPanels`    | `ReactNode`                 | ビューア下部に追加するパネル                     |
| `extraOverlay`   | `ReactNode`                 | ビューア上に絶対配置するオーバーレイ要素         |

### SideAccordion の使い方

`AccordionItem` の配列を渡すだけで複数展開可能なアコーディオンになる。`RobotPageLayout` を使う場合は自動的に内包されるため、直接使用するのは `RobotPageLayout` を使わないページのみ。

```ts
export interface AccordionItem {
  id: string;
  label: string;
  children: ReactNode;
}
```

## 3D ビューア（RosViewer）

`RobotPageLayout` の `viewerMode` を指定すると右側に `RosViewer` が表示される。表示レイヤーは `VisualizationContext` の `layers` で制御され、ユーザーが Settings で切り替え可能。

### レイヤー一覧（`LayerKey`）

| キー                | 内容                                   |
| ------------------- | -------------------------------------- |
| `map`               | 占有格子地図                           |
| `globalCostmap`     | グローバルコストマップ                 |
| `localCostmap`      | ローカルコストマップ                   |
| `lidarTop`          | 上部LiDARスキャン（シアン）            |
| `lidarFront`        | 前部LiDARスキャン（緑）                |
| `robotPose`         | ロボット位置（矢印）                   |
| `particleCloud`     | AMCLパーティクル                       |
| `planPath`          | ナビゲーション計画パス（赤）           |
| `actualPath`        | 実走パス（紫）                         |
| `waypointMarkers`   | ウェイポイントマーカー                 |
| `collisionPolygons` | 衝突判定ポリゴン                       |
| `pointCloud`        | 3D点群                                 |
| `colorImage`        | カラーカメラ画像（ビューア下部に表示） |
| `depthImage`        | デプス画像（ビューア下部に表示）       |

### オーバーレイ一覧（`OverlayKey`）

| キー            | 内容                       |
| --------------- | -------------------------- |
| `joystick`      | 右下ジョイスティックパッド |
| `velocityGauge` | 左上速度ゲージ             |
| `systemMetrics` | 左上システムメトリクス     |

### 新しいレイヤーを追加する手順

1. `contexts/VisualizationContext.tsx` の `LayerKey` Union 型と `DEFAULT_LAYERS` に追加
2. `components/ros-viewer/RosViewer.tsx` の `Scene` コンポーネント内で `{layers.myLayer && <MyLayer client={client} />}` を追加
3. `components/ros-viewer/layers/MyLayer.tsx` を作成

Layerコンポーネントのパターン:

```tsx
// layers/MyLayer.tsx
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../../hooks/useTopicSubscriber";
import { TOPICS } from "../../../ros/interfaces";
import { TfBuffer } from "../hooks/useTfBuffer";

export default function MyLayer({
  client,
  tfBuffer,
}: {
  client: FoxgloveClientHandle;
  tfBuffer: TfBuffer; // 座標変換が必要な場合
}) {
  const data = useTopicSubscriber<MyMsg>(
    client,
    TOPICS.MY_TOPIC,
    "pkg/msg/MyMsg",
  );
  if (!data) return null;
  // Three.js コンポーネントを返す（<mesh>, <points> 等）
  return <mesh>...</mesh>;
}
```

- レイヤーコンポーネントは `@react-three/fiber` の `Canvas` 内でレンダリングされるため、戻り値は Three.js プリミティブ（`<mesh>`, `<points>` 等）にする
- `tfBuffer.lookupTransform(targetFrame, sourceFrame)` で `THREE.Matrix4 | null` を取得できる
- `ros-viewer/hooks/` に既存の専用フック（`useLaserScan`, `useOccupancyGrid`, `usePath` 等）がある場合はそれを使う

## 型定義の場所

| 種別                | ファイル                                                        |
| ------------------- | --------------------------------------------------------------- |
| ROSメッセージ型     | [types/ros.ts](../../mg_ui/mg_web_ui/frontend/src/types/ros.ts) |
| SystemManager API型 | [types/api.ts](../../mg_ui/mg_web_ui/frontend/src/types/api.ts) |
| 再エクスポート      | `types.ts` → `export * from './types/ros'`                      |

`types/ros.ts` のパターン:

```ts
export interface MyMsg {
  header: Header;
  data: number[] | Float32Array; // バイナリ配列はUnion型
  child_frame_id: string;
}
```

## ROSファイル定義の参照先

| ファイル                                                                  | 役割                          |
| ------------------------------------------------------------------------- | ----------------------------- |
| [ros/topics.ts](../../mg_ui/mg_web_ui/frontend/src/ros/topics.ts)         | トピック名定数                |
| [ros/services.ts](../../mg_ui/mg_web_ui/frontend/src/ros/services.ts)     | サービス名定数                |
| [ros/schemas.ts](../../mg_ui/mg_web_ui/frontend/src/ros/schemas.ts)       | publish用スキーマ定義         |
| [ros/namespaces.ts](../../mg_ui/mg_web_ui/frontend/src/ros/namespaces.ts) | NODE_NS定数 + nodeNs()関数    |
| [ros/interfaces.ts](../../mg_ui/mg_web_ui/frontend/src/ros/interfaces.ts) | 上記4ファイルの再エクスポート |
