import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useSimulation } from "../contexts/SimulationContext";
import RobotPageLayout from "../components/RobotPageLayout";
import ApiLogSection from "../components/sections/ApiLogSection";
import ContainerStatusCard from "../components/sections/ContainerStatusCard";
import ServiceControlCard from "../components/sections/ServiceControlCard";
import SectionCard from "../components/layout/SectionCard";
import SimulationPoseSection from "../components/sections/SimulationPoseSection";

export default function SlamPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [mapDir, setMapDir] = useState("/root/ros2_data");
  const [mapName, setMapName] = useState("map");
  const { isSimulation } = useSimulation();

  const { containers, callApi } = sysManager;
  const slamState = containers["slam"] ?? "unknown";

  const call = async (path: string, body?: unknown) => {
    setLoading(true);
    setError(null);
    try {
      const result = await callApi(path, body);
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
          <ContainerStatusCard title="SLAM" status={slamState} />
          <ServiceControlCard
            title="Control"
            buttons={[
              slamState === "running"
                ? {
                    label: "Restart SLAM",
                    onClick: () => call("/slam/restart"),
                    variant: "blue" as const,
                  }
                : {
                    label: "Start SLAM",
                    onClick: () => call("/slam/start"),
                    variant: "green" as const,
                  },
              {
                label: "Stop SLAM",
                onClick: () => call("/slam/stop"),
                variant: "red",
              },
            ]}
            loading={loading}
            error={error}
          />
          <SectionCard title="Save Map">
            <div className="space-y-2">
              <div>
                <label className="text-xs text-gray-400 block mb-1">Directory</label>
                <input
                  type="text"
                  value={mapDir}
                  onChange={(e) => setMapDir(e.target.value)}
                  className="w-full bg-gray-700 text-sm text-white px-2 py-1 rounded border border-gray-600 focus:outline-none focus:border-blue-500"
                />
              </div>
              <div>
                <label className="text-xs text-gray-400 block mb-1">Map Name</label>
                <input
                  type="text"
                  value={mapName}
                  onChange={(e) => setMapName(e.target.value)}
                  className="w-full bg-gray-700 text-sm text-white px-2 py-1 rounded border border-gray-600 focus:outline-none focus:border-blue-500"
                />
              </div>
              <button
                onClick={() => call("/map/save", { map_dir: mapDir, map_name: mapName })}
                disabled={loading}
                className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
              >
                Save Map
              </button>
            </div>
          </SectionCard>
        </div>
      ),
    },
    ...(isSimulation
      ? [
          {
            id: "simulation",
            label: "Simulation",
            children: (
              <SimulationPoseSection
                onResetRobot={(pose) => call("/simulation/reset-pose")}
                loading={loading}
              />
            ),
          },
        ]
      : []),
    {
      id: "log",
      label: "Log",
      children: <ApiLogSection logs={sysManager.logs} />,
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
