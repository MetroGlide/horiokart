import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useSimulation } from "../contexts/SimulationContext";
import RobotPageLayout from "../components/RobotPageLayout";
import ApiLogSection from "../components/sections/ApiLogSection";
import ContainerStatusCard from "../components/sections/ContainerStatusCard";
import ServiceControlCard from "../components/sections/ServiceControlCard";
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
          <ContainerStatusCard title="SLAM Container" status={slamState} />
          <ServiceControlCard
            title="SLAM Control"
            buttons={[
              {
                label: "Start SLAM",
                onClick: () => call("/slam/start"),
                variant: "green",
              },
              {
                label: "Stop SLAM",
                onClick: () => call("/slam/stop"),
                variant: "red",
              },
              {
                label: "Save Map",
                onClick: () => call("/map/save"),
                variant: "blue",
              },
            ]}
            loading={loading}
            error={error}
          />
        </div>
      ),
    },
    ...(isSimulation
      ? [
          {
            id: "simulation",
            label: "シミュレーション",
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
      label: "ログ",
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
