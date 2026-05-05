import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useSimulation } from "../contexts/SimulationContext";
import ApiLogPanel from "../components/ApiLogPanel";
import SectionCard from "../components/SectionCard";
import RobotPageLayout from "../components/RobotPageLayout";

export default function SlamPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const { isSimulation } = useSimulation();

  const { containers, callApi } = sysManager;
  const slamState = containers["slam"] ?? "unknown";

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

  const accordionItems = [
    {
      id: "slam",
      label: "SLAM",
      children: (
        <div className="space-y-2">
          <SectionCard title="SLAM Container">
            <div className="flex items-center gap-3">
              <span
                className={`w-3 h-3 rounded-full ${slamState === "running" ? "bg-green-500" : "bg-gray-500"}`}
              />
              <span className="text-lg font-semibold capitalize">
                {slamState}
              </span>
            </div>
          </SectionCard>
          <SectionCard title="SLAM Control">
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
          </SectionCard>
        </div>
      ),
    },
    ...(isSimulation
      ? [
          {
            id: "simulation",
            label: "シミュレーション",
            children: (
              <div className="flex flex-wrap gap-3">
                <button
                  onClick={() => call("/simulation/reset-pose")}
                  disabled={loading}
                  className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
                >
                  Reset Robot Pose
                </button>
              </div>
            ),
          },
        ]
      : []),
    {
      id: "log",
      label: "ログ",
      children: <ApiLogPanel logs={sysManager.logs} />,
    },
  ];

  return (
    <RobotPageLayout
      client={client}
      accordionItems={accordionItems}
      defaultOpen={["slam", ...(isSimulation ? ["simulation"] : [])]}
      viewerMode="2d"
    />
  );
}
