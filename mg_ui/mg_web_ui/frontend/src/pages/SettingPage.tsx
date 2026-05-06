import { useSimulation } from "../contexts/SimulationContext";
import {
  useVisualization,
  LayerKey,
  OverlayKey,
} from "../contexts/VisualizationContext";
import { useTeleop } from "../contexts/TeleopContext";
import Toggle from "../components/ui/Toggle";

interface LayerGroup {
  label: string;
  items: { key: LayerKey; label: string }[];
}

const LAYER_GROUPS: LayerGroup[] = [
  {
    label: "Map",
    items: [
      { key: "map", label: "Map" },
      { key: "globalCostmap", label: "Global Costmap" },
      { key: "localCostmap", label: "Local Costmap" },
    ],
  },
  {
    label: "LiDAR",
    items: [
      { key: "lidarTop", label: "Top LiDAR" },
      { key: "lidarFront", label: "Front LiDAR" },
    ],
  },
  {
    label: "Robot",
    items: [
      { key: "robotPose", label: "Robot Pose (AMCL)" },
      { key: "particleCloud", label: "Particle Cloud" },
    ],
  },
  {
    label: "Navigation",
    items: [
      { key: "planPath", label: "Planned Path" },
      { key: "actualPath", label: "Actual Path" },
      { key: "waypointMarkers", label: "Waypoint Markers" },
      { key: "collisionPolygons", label: "Collision Polygons" },
    ],
  },
  {
    label: "Sensors",
    items: [
      { key: "pointCloud", label: "Point Cloud (Depth)" },
      { key: "cameraImage", label: "Camera Image" },
    ],
  },
];

interface OverlayItem {
  key: OverlayKey;
  label: string;
  description: string;
}

const OVERLAY_CONFIG: OverlayItem[] = [
  {
    key: "joystick",
    label: "Joystick Pad",
    description: "操作パッド（選択式、デフォルトOFF）",
  },
  {
    key: "velocityGauge",
    label: "Velocity Gauge",
    description: "速度指令値・実測値メーター",
  },
  {
    key: "systemMetrics",
    label: "System Metrics",
    description: "CPU / メモリ使用率",
  },
];

export default function SettingPage() {
  const { isSimulation, setIsSimulation } = useSimulation();
  const { enabled, layers, overlays, setEnabled, toggleLayer, toggleOverlay } =
    useVisualization();
  const {
    maxLinear,
    maxAngular,
    gaugeMaxLinear,
    gaugeMaxAngular,
    gaugeSyncWithPad,
    setMaxLinear,
    setMaxAngular,
    setGaugeMaxLinear,
    setGaugeMaxAngular,
    setGaugeSyncWithPad,
  } = useTeleop();

  return (
    <div className="space-y-6 max-w-2xl mx-auto">
      <section className="bg-gray-800 rounded-lg p-4 space-y-4">
        <p className="text-xs text-gray-400">Simulation</p>
        <div className="flex items-center gap-4">
          <span className="text-sm text-gray-300">Simulation Mode</span>
          <Toggle
            value={isSimulation}
            onChange={() => setIsSimulation(!isSimulation)}
          />
          <span
            className={`text-sm font-semibold ${isSimulation ? "text-blue-400" : "text-gray-500"}`}
          >
            {isSimulation ? "ON" : "OFF"}
          </span>
        </div>
        <p className="text-xs text-gray-500">
          Enable to show simulation-related features on each page.
        </p>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-4">
        <div className="flex items-center justify-between">
          <p className="text-xs text-gray-400">Visualization</p>
        </div>
        <div className="flex items-center gap-4">
          <span className="text-sm text-gray-300">Enable Visualization</span>
          <Toggle value={enabled} onChange={() => setEnabled(!enabled)} />
          <span
            className={`text-sm font-semibold ${enabled ? "text-blue-400" : "text-gray-500"}`}
          >
            {enabled ? "ON" : "OFF"}
          </span>
        </div>
        <p className="text-xs text-gray-500">
          Disable to stop all visualization topic subscriptions.
        </p>

        {enabled && (
          <div className="space-y-4 pt-2 border-t border-gray-700">
            {LAYER_GROUPS.map((group) => (
              <div key={group.label}>
                <p className="text-xs text-gray-400 mb-2">{group.label}</p>
                <div className="space-y-2">
                  {group.items.map(({ key, label }) => (
                    <div key={key} className="flex items-center gap-3">
                      <Toggle
                        value={layers[key]}
                        onChange={() => toggleLayer(key)}
                      />
                      <span className="text-sm text-gray-300">{label}</span>
                    </div>
                  ))}
                </div>
              </div>
            ))}
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-4">
        <p className="text-xs text-gray-400">Viewer Overlays</p>
        <p className="text-xs text-gray-500">
          可視化領域に重ねて表示する要素を設定します。
        </p>
        <div className="space-y-2">
          {OVERLAY_CONFIG.map(({ key, label, description }) => (
            <div key={key} className="flex items-center gap-3">
              <Toggle
                value={overlays[key]}
                onChange={() => toggleOverlay(key)}
              />
              <div>
                <span className="text-sm text-gray-300">{label}</span>
                <p className="text-xs text-gray-500">{description}</p>
              </div>
            </div>
          ))}
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-4">
        <p className="text-xs text-gray-400">Teleop</p>
        <div>
          <p className="text-xs text-gray-400 mb-2">Pad speed limit</p>
          <div className="grid grid-cols-2 gap-4">
            <label className="flex flex-col gap-1">
              <span className="text-sm text-gray-300">Max Linear (m/s)</span>
              <input
                type="number"
                min={0.1}
                max={2.0}
                step={0.1}
                value={maxLinear}
                onChange={(e) => setMaxLinear(Number(e.target.value))}
                className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
              />
            </label>
            <label className="flex flex-col gap-1">
              <span className="text-sm text-gray-300">Max Angular (rad/s)</span>
              <input
                type="number"
                min={0.1}
                max={2.0}
                step={0.1}
                value={maxAngular}
                onChange={(e) => setMaxAngular(Number(e.target.value))}
                className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
              />
            </label>
          </div>
        </div>
        <div className="pt-3 border-t border-gray-700 space-y-3">
          <div className="flex items-center gap-3">
            <Toggle
              value={gaugeSyncWithPad}
              onChange={() => setGaugeSyncWithPad(!gaugeSyncWithPad)}
            />
            <div>
              <span className="text-sm text-gray-300">
                Gauge: Sync with pad limits
              </span>
              <p className="text-xs text-gray-500">
                When ON, gauge display limit equals pad speed limit.
              </p>
            </div>
          </div>
          {!gaugeSyncWithPad && (
            <div>
              <p className="text-xs text-gray-400 mb-2">Gauge display limit</p>
              <div className="grid grid-cols-2 gap-4">
                <label className="flex flex-col gap-1">
                  <span className="text-sm text-gray-300">
                    Max Linear (m/s)
                  </span>
                  <input
                    type="number"
                    min={0.1}
                    max={5.0}
                    step={0.1}
                    value={gaugeMaxLinear}
                    onChange={(e) => setGaugeMaxLinear(Number(e.target.value))}
                    className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
                  />
                </label>
                <label className="flex flex-col gap-1">
                  <span className="text-sm text-gray-300">
                    Max Angular (rad/s)
                  </span>
                  <input
                    type="number"
                    min={0.1}
                    max={5.0}
                    step={0.1}
                    value={gaugeMaxAngular}
                    onChange={(e) => setGaugeMaxAngular(Number(e.target.value))}
                    className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
                  />
                </label>
              </div>
            </div>
          )}
        </div>
      </section>
    </div>
  );
}
