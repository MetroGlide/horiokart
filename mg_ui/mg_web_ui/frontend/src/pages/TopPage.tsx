import { useNavigate } from "react-router-dom";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useDiagnosticsMap } from "../hooks/useDiagnosticsMap";
import { SequencerStatus, DIAG_COLOR, DIAG_LEVEL } from "../types";

export default function TopPage({ client }: { client: FoxgloveClientHandle }) {
  const navigate = useNavigate();
  const seqStatus = useTopicSubscriber<SequencerStatus>(
    client,
    "waypoint_sequencer_node/status",
    "mg_msgs/msg/SequencerStatus",
  );
  const diagStatuses = useDiagnosticsMap(client);

  const warnCount = diagStatuses.filter((s) => s.level >= 1).length;
  const errorCount = diagStatuses.filter((s) => s.level >= 2).length;

  const panels = [
    { label: "Waypoint Nav", to: "/waypoint", icon: "🗺️" },
    { label: "SLAM", to: "/slam", icon: "📡" },
    { label: "Simulation", to: "/simulation", icon: "🧪" },
    { label: "Utility", to: "/utility", icon: "🛠️" },
  ];

  return (
    <div className="space-y-6">
      <section className="grid grid-cols-2 gap-4">
        <div className="bg-gray-800 rounded-lg p-4">
          <p className="text-xs text-gray-400 mb-1">Sequencer State</p>
          <p className="text-2xl font-bold">{seqStatus?.state ?? "—"}</p>
          {seqStatus && (
            <p className="text-sm text-gray-400 mt-1">
              {Math.min(seqStatus.current_index + 1, seqStatus.total_waypoints)}{" "}
              / {seqStatus.total_waypoints} pts
              {seqStatus.distance_remaining > 0 &&
                ` · ${seqStatus.distance_remaining.toFixed(1)} m`}
            </p>
          )}
        </div>
        <div className="bg-gray-800 rounded-lg p-4">
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
          <ul className="space-y-1">
            {diagStatuses
              .filter((s) => s.level >= 1)
              .map((s) => (
                <li key={s.name} className="flex gap-2 text-sm">
                  <span className={`font-semibold ${DIAG_COLOR[s.level]}`}>
                    {DIAG_LEVEL[s.level]}
                  </span>
                  <span className="text-gray-300">{s.name}</span>
                  <span className="text-gray-500">{s.message}</span>
                </li>
              ))}
          </ul>
        </section>
      )}

      <section className="grid grid-cols-2 md:grid-cols-4 gap-4">
        {panels.map(({ label, to, icon }) => (
          <button
            key={to}
            onClick={() => navigate(to)}
            className="bg-gray-800 hover:bg-gray-700 rounded-lg p-6 text-center transition-colors"
          >
            <div className="text-3xl mb-2">{icon}</div>
            <div className="font-medium">{label}</div>
          </button>
        ))}
      </section>
    </div>
  );
}
