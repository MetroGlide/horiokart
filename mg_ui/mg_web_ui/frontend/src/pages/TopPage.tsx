import { useNavigate } from "react-router-dom";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useDiagnosticsMap } from "../hooks/useDiagnosticsMap";
import { useVisualization } from "../contexts/VisualizationContext";
import SystemMetrics from "../components/SystemMetrics";
import DiagnosticsTable from "../components/panels/DiagnosticsTable";

const navCards = [
  {
    label: "Waypoint Nav",
    to: "/waypoint",
    icon: "🗺️",
    description: "Waypoint navigation control",
  },
  {
    label: "SLAM",
    to: "/slam",
    icon: "📡",
    description: "SLAM, map saving, localization",
  },
  {
    label: "System",
    to: "/system",
    icon: "🛠️",
    description: "Service management & diagnostics",
  },
  {
    label: "Setting",
    to: "/setting",
    icon: "⚙️",
    description: "Simulation mode, display settings",
  },
];

export default function TopPage({ client }: { client: FoxgloveClientHandle }) {
  const navigate = useNavigate();
  const diagStatuses = useDiagnosticsMap(client);
  const { overlays } = useVisualization();

  const warnCount = diagStatuses.filter((s) => s.level >= 1).length;
  const errorCount = diagStatuses.filter((s) => s.level >= 2).length;

  return (
    <div className="space-y-6">
      <section className="grid grid-cols-2 gap-4">
        {navCards.map(({ label, to, icon, description }) => (
          <button
            key={to}
            onClick={() => navigate(to)}
            className="bg-gray-800 hover:bg-gray-700 rounded-lg p-6 text-left transition-colors border border-gray-700 hover:border-gray-500"
          >
            <div className="text-3xl mb-2">{icon}</div>
            <div className="font-semibold text-lg">{label}</div>
            <div className="text-xs text-gray-400 mt-1">{description}</div>
          </button>
        ))}
      </section>

      <section className="grid grid-cols-2 gap-4">
        <div className="bg-gray-800 rounded-lg p-4 col-span-2">
          <p className="text-xs text-gray-400 mb-1">System Health</p>
          {errorCount > 0 ? (
            <p className="text-2xl font-bold text-red-400">
              {errorCount} ERROR
            </p>
          ) : warnCount > 0 ? (
            <p className="text-2xl font-bold text-yellow-400">
              {warnCount} WARN
            </p>
          ) : (
            <p className="text-2xl font-bold text-green-400">OK</p>
          )}
          <p className="text-sm text-gray-400 mt-1">
            {diagStatuses.length > 0
              ? `${diagStatuses.length} items monitored`
              : "waiting…"}
          </p>
        </div>
      </section>

      {warnCount + errorCount > 0 && (
        <section className="bg-gray-800 rounded-lg p-4">
          <p className="text-xs text-gray-400 mb-2">Alerts</p>
          <DiagnosticsTable
            statuses={diagStatuses.filter((s) => s.level >= 1)}
            compact
          />
        </section>
      )}

      {overlays.systemMetrics && (
        <section className="bg-gray-800 rounded-lg p-4">
          <p className="text-xs text-gray-400 mb-2">System Resources</p>
          <SystemMetrics client={client} />
        </section>
      )}
    </div>
  );
}
