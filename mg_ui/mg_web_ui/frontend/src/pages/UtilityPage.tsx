import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useServiceCaller } from "../hooks/useServiceCaller";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { DiagnosticArray, DIAG_COLOR, DIAG_LEVEL } from "../types";

export default function UtilityPage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const { call, loading, error } = useServiceCaller(client);
  const diagnostics = useTopicSubscriber<DiagnosticArray>(
    client,
    "/diagnostics",
    "diagnostic_msgs/msg/DiagnosticArray",
  );

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Tools</p>
        <div className="flex flex-wrap gap-3">
          <button
            onClick={() =>
              call("system_manager_node/start_waypoint_editor", {})
            }
            disabled={loading}
            className="bg-gray-600 hover:bg-gray-500 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Launch Waypoint Editor
          </button>
        </div>
        {error && <p className="text-red-400 text-sm">{error}</p>}
      </section>

      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-3">Diagnostics Detail</p>
        {diagnostics ? (
          <table className="w-full text-sm">
            <thead>
              <tr className="text-gray-400 text-left border-b border-gray-700">
                <th className="pb-2 w-16">Level</th>
                <th className="pb-2">Name</th>
                <th className="pb-2">Message</th>
              </tr>
            </thead>
            <tbody>
              {diagnostics.status.map((s) => (
                <tr key={s.name} className="border-b border-gray-700/50">
                  <td className={`py-2 font-semibold ${DIAG_COLOR[s.level]}`}>
                    {DIAG_LEVEL[s.level]}
                  </td>
                  <td className="py-2 text-gray-300">{s.name}</td>
                  <td className="py-2 text-gray-500">{s.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <p className="text-sm text-gray-500">waiting for /diagnostics…</p>
        )}
      </section>
    </div>
  );
}
