import { useState } from "react";
import { Canvas } from "@react-three/fiber";
import { MapControls } from "@react-three/drei";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { TOPICS } from "../ros/topics";
import { useGpsFix } from "../hooks/useGpsFix";
import MapLayer from "../components/ros-viewer/layers/MapLayer";
import PathLine from "../components/ros-viewer/layers/PathLine";
import MarkerArrayLayer from "../components/ros-viewer/layers/MarkerArrayLayer";
import GpsMapOverlay from "../components/panels/GpsMapOverlay";

interface LayerState {
  poseGraph: boolean;
  gnssRaw: boolean;
  gnssPrior: boolean;
  pathBefore: boolean;
}

interface ToggleProps {
  label: string;
  checked: boolean;
  onChange: (v: boolean) => void;
  color: string;
}

function LayerToggle({ label, checked, onChange, color }: ToggleProps) {
  return (
    <label className="flex items-center gap-1.5 cursor-pointer select-none">
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="w-3.5 h-3.5 rounded"
      />
      <span
        className="text-xs font-mono"
        style={{ color: checked ? color : "#6b7280" }}
      >
        {label}
      </span>
    </label>
  );
}

export default function SlamGnss2DPage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [layers, setLayers] = useState<LayerState>({
    poseGraph: true,
    gnssRaw: true,
    gnssPrior: true,
    pathBefore: true,
  });
  const { fix, trail } = useGpsFix(client);

  const toggle = (key: keyof LayerState) => (v: boolean) =>
    setLayers((prev) => ({ ...prev, [key]: v }));

  return (
    <div className="flex flex-col gap-2">
      {/* レイヤー切替 */}
      <div className="flex flex-wrap items-center gap-4 px-3 py-2 bg-gray-800 rounded border border-gray-700">
        <span className="text-xs text-gray-400 font-semibold uppercase tracking-wide">
          Layers
        </span>
        <LayerToggle
          label="Pose Graph"
          checked={layers.poseGraph}
          onChange={toggle("poseGraph")}
          color="#60a5fa"
        />
        <LayerToggle
          label="GNSS Points"
          checked={layers.gnssRaw}
          onChange={toggle("gnssRaw")}
          color="#e879f9"
        />
        <LayerToggle
          label="GNSS Constraints"
          checked={layers.gnssPrior}
          onChange={toggle("gnssPrior")}
          color="#c084fc"
        />
        <LayerToggle
          label="Pre-optimize Path"
          checked={layers.pathBefore}
          onChange={toggle("pathBefore")}
          color="#6b7280"
        />
      </div>

      {/* メインエリア */}
      <div className="flex gap-2" style={{ height: "calc(100vh - 160px)" }}>
        {/* SLAM キャンバス */}
        <div className="flex-1 rounded overflow-hidden bg-gray-900 border border-gray-700">
          <Canvas
            orthographic
            camera={{ zoom: 10, position: [0, 0, 100], up: [0, 1, 0] }}
            gl={{ antialias: false }}
          >
            <ambientLight intensity={1} />
            <MapControls screenSpacePanning makeDefault />
            <MapLayer client={client} topic={TOPICS.SLAM_GNSS2D_MAP} />
            <PathLine
              client={client}
              topic={TOPICS.SLAM_GNSS2D_PATH}
              color="#00ffff"
              lineWidth={2}
            />
            {layers.pathBefore && (
              <PathLine
                client={client}
                topic={TOPICS.SLAM_GNSS2D_PATH_BEFORE}
                color="#666666"
                lineWidth={1}
              />
            )}
            {layers.poseGraph && (
              <MarkerArrayLayer
                client={client}
                topic={TOPICS.SLAM_GNSS2D_POSE_GRAPH}
              />
            )}
            {layers.gnssRaw && (
              <MarkerArrayLayer
                client={client}
                topic={TOPICS.SLAM_GNSS2D_GNSS_RAW}
              />
            )}
            {layers.gnssPrior && (
              <MarkerArrayLayer
                client={client}
                topic={TOPICS.SLAM_GNSS2D_GNSS_PRIOR}
              />
            )}
          </Canvas>
        </div>

        {/* 衛星地図パネル */}
        <div className="flex flex-col gap-2 w-56">
          <div className="text-xs text-gray-400 font-semibold uppercase tracking-wide px-1">
            GPS Map
          </div>
          <GpsMapOverlay
            fix={fix}
            trail={trail}
            mapWidth={224}
            mapHeight={224}
          />
        </div>
      </div>
    </div>
  );
}
