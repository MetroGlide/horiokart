import { useState } from "react";
import { useSimulation } from "../contexts/SimulationContext";
import { useRosbagReplay } from "../contexts/RosbagReplayContext";
import {
  useVisualization,
  LayerKey,
  OverlayKey,
} from "../contexts/VisualizationContext";
import { useTeleop } from "../contexts/TeleopContext";
import Toggle from "../components/ui/Toggle";
import {
  getSysManagerUrl,
  getSysManagerDefaultUrl,
  setSysManagerUrl,
  resetSysManagerUrl,
} from "../utils/systemManagerConfig";

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
      { key: "colorImage", label: "Color Image" },
      { key: "depthImage", label: "Depth Image" },
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
  const { isRosbagReplayVisible, setIsRosbagReplayVisible } = useRosbagReplay();
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

  const [openSections, setOpenSections] = useState<Set<string>>(
    () => new Set(),
  );

  const [sysManagerUrl, setSysManagerUrlState] = useState(() =>
    getSysManagerUrl(),
  );

  const handleSysManagerUrlBlur = () => {
    setSysManagerUrl(sysManagerUrl);
  };

  const handleSysManagerUrlReset = () => {
    resetSysManagerUrl();
    setSysManagerUrlState(getSysManagerDefaultUrl());
  };

  const toggleSection = (id: string) => {
    setOpenSections((prev) => {
      const next = new Set(prev);
      if (next.has(id)) next.delete(id);
      else next.add(id);
      return next;
    });
  };

  const sectionHeader = (id: string, label: string) => (
    <button
      onClick={() => toggleSection(id)}
      className="w-full flex items-center justify-between text-xs font-semibold text-gray-400 hover:text-gray-200 transition-colors"
    >
      <span>{label}</span>
      <svg
        className={`w-4 h-4 flex-shrink-0 transform transition-transform ${
          openSections.has(id) ? "rotate-90" : ""
        }`}
        fill="none"
        viewBox="0 0 24 24"
        stroke="currentColor"
      >
        <path
          strokeLinecap="round"
          strokeLinejoin="round"
          strokeWidth={2}
          d="M9 5l7 7-7 7"
        />
      </svg>
    </button>
  );

  return (
    <div className="space-y-3 max-w-2xl mx-auto">
      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("connection", "Connection")}
        {openSections.has("connection") && (
          <div className="space-y-3 pt-1">
            <div>
              <label className="text-xs text-gray-400 block mb-1">
                System Manager URL
              </label>
              <div className="flex gap-2">
                <input
                  type="text"
                  value={sysManagerUrl}
                  onChange={(e) => setSysManagerUrlState(e.target.value)}
                  onBlur={handleSysManagerUrlBlur}
                  className="flex-1 bg-gray-700 text-sm text-white px-2 py-1 rounded border border-gray-600 focus:outline-none focus:border-blue-500"
                />
                <button
                  onClick={handleSysManagerUrlReset}
                  className="px-3 py-1 rounded text-sm font-medium bg-gray-600 hover:bg-gray-500 text-gray-200 flex-shrink-0"
                >
                  Reset
                </button>
              </div>
              <p className="text-xs text-gray-500 mt-1">
                Default: {getSysManagerDefaultUrl()}
              </p>
            </div>
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("simulation", "Simulation")}
        {openSections.has("simulation") && (
          <div className="space-y-4 pt-1">
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
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("rosbag-replay", "Rosbag Replay")}
        {openSections.has("rosbag-replay") && (
          <div className="space-y-4 pt-1">
            <div className="flex items-center gap-4">
              <span className="text-sm text-gray-300">Rosbag Replay</span>
              <Toggle
                value={isRosbagReplayVisible}
                onChange={() =>
                  setIsRosbagReplayVisible(!isRosbagReplayVisible)
                }
              />
              <span
                className={`text-sm font-semibold ${isRosbagReplayVisible ? "text-blue-400" : "text-gray-500"}`}
              >
                {isRosbagReplayVisible ? "ON" : "OFF"}
              </span>
            </div>
            <p className="text-xs text-gray-500">
              Enable to show rosbag replay controls on each page.
            </p>
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("visualization", "Visualization")}
        {openSections.has("visualization") && (
          <div className="space-y-4 pt-1">
            <div className="flex items-center gap-4">
              <span className="text-sm text-gray-300">
                Enable Visualization
              </span>
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
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("overlays", "Viewer Overlays")}
        {openSections.has("overlays") && (
          <div className="space-y-4 pt-1">
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
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        {sectionHeader("teleop", "Teleop")}
        {openSections.has("teleop") && (
          <div className="space-y-4 pt-1">
            <div>
              <p className="text-xs text-gray-400 mb-2">Pad speed limit</p>
              <div className="grid grid-cols-2 gap-4">
                <label className="flex flex-col gap-1">
                  <span className="text-sm text-gray-300">
                    Max Linear (m/s)
                  </span>
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
                  <span className="text-sm text-gray-300">
                    Max Angular (rad/s)
                  </span>
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
                  <p className="text-xs text-gray-400 mb-2">
                    Gauge display limit
                  </p>
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
                        onChange={(e) =>
                          setGaugeMaxLinear(Number(e.target.value))
                        }
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
                        onChange={(e) =>
                          setGaugeMaxAngular(Number(e.target.value))
                        }
                        className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
                      />
                    </label>
                  </div>
                </div>
              )}
            </div>
          </div>
        )}
      </section>
    </div>
  );
}
