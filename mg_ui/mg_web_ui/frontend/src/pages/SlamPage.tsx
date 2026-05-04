import { useState } from "react";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { DiagnosticArray } from "../types";
import ApiLogPanel from "../components/ApiLogPanel";

export default function SlamPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const diagnostics = useTopicSubscriber<DiagnosticArray>(
    client,
    "/diagnostics",
    "diagnostic_msgs/msg/DiagnosticArray",
  );

  const { containers, callApi } = sysManager;
  const slamState = containers["slam"] ?? "unknown";

  const amclStatus = diagnostics?.status.find(
    (s) => s.name === "localization/amcl_covariance",
  );
  const traceXy = amclStatus?.values.find((v) => v.key === "trace_xy")?.value;

  const call = async (path: string) => {
    setLoading(true);
    setError(null);
    try {
      const result = await callApi(path);
      if (!result.success) setError(result.message);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-2">SLAM Container</p>
        <div className="flex items-center gap-3">
          <span
            className={`w-3 h-3 rounded-full ${
              slamState === "running" ? "bg-green-500" : "bg-gray-500"
            }`}
          />
          <span className="text-lg font-semibold capitalize">{slamState}</span>
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-1">AMCL Localization Quality</p>
        {amclStatus ? (
          <div>
            <p className="text-sm text-gray-300">{amclStatus.message}</p>
            {traceXy && (
              <p className="text-xs text-gray-500 mt-1">trace_xy = {traceXy}</p>
            )}
          </div>
        ) : (
          <p className="text-sm text-gray-500">waiting…</p>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">SLAM Control</p>
        <div className="flex flex-wrap gap-3">
          <button
            onClick={() => call("/slam/start")}
            disabled={loading}
            className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Start SLAM
          </button>
          <button
            onClick={() => call("/slam/stop")}
            disabled={loading}
            className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Stop SLAM
          </button>
          <button
            onClick={() => call("/map/save")}
            disabled={loading}
            className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Save Map
          </button>
        </div>
        {error && <p className="text-red-400 text-sm">{error}</p>}
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Navigation Control</p>
        <div className="flex flex-wrap gap-3">
          <button
            onClick={() => call("/navigation/start")}
            disabled={loading}
            className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Start Navigation
          </button>
          <button
            onClick={() => call("/navigation/stop")}
            disabled={loading}
            className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Stop Navigation
          </button>
        </div>
      </section>
      <ApiLogPanel logs={sysManager.logs} />
    </div>
  );
}
