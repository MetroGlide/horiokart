import { useSimulation } from "../contexts/SimulationContext";
import { useVisualization, LayerKey } from "../contexts/VisualizationContext";

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

function Toggle({ value, onChange }: { value: boolean; onChange: () => void }) {
  return (
    <button
      onClick={onChange}
      className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors ${
        value ? "bg-blue-600" : "bg-gray-600"
      }`}
    >
      <span
        className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
          value ? "translate-x-6" : "translate-x-1"
        }`}
      />
    </button>
  );
}

export default function SettingPage() {
  const { isSimulation, setIsSimulation } = useSimulation();
  const { enabled, layers, setEnabled, toggleLayer } = useVisualization();

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
          ON にすると各ページの Simulation
          関連機能が表示されます。設定は保持されます。
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
          OFF にすると RViz
          ビューア全体が無効化され、関連トピックの購読が停止します。
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
    </div>
  );
}
