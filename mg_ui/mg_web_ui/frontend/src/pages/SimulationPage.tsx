import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import ApiLogPanel from "../components/ApiLogPanel";

interface PoseInput {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

function buildInitialPoseMessage(pose: PoseInput) {
  const nowMs = Date.now();
  const sec = Math.floor(nowMs / 1000);
  const nanosec = Math.floor((nowMs % 1000) * 1_000_000);
  const qz = Math.sin(pose.yaw / 2.0);
  const qw = Math.cos(pose.yaw / 2.0);
  const covariance = Array(36).fill(0.0);
  covariance[0] = 0.25;
  covariance[7] = 0.25;
  covariance[35] = 0.06853891945200942;

  return {
    header: {
      stamp: { sec, nanosec },
      frame_id: "map",
    },
    pose: {
      pose: {
        position: { x: pose.x, y: pose.y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
      },
      covariance,
    },
  };
}

export default function SimulationPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [pose, setPose] = useState<PoseInput>({
    x: 0.0,
    y: 0.0,
    z: 0.05,
    yaw: 0.0,
  });

  const { containers, callApi } = sysManager;
  const scenarioState = containers["scenario-test"] ?? "unknown";

  const updatePose = (key: keyof PoseInput, value: number) => {
    setPose((prev) => ({ ...prev, [key]: value }));
  };

  const callSys = async (path: string, body?: unknown) => {
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

  const handleResetRobotPose = () => callSys("/simulation/reset-pose", pose);

  const handleResetAmclPose = () => {
    setError(null);
    try {
      client.publish(
        "/initialpose",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        buildInitialPoseMessage(pose),
      );
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    }
  };

  const handleStartScenario = () => callSys("/scenario-test/start");
  const handleStopScenario = () => callSys("/scenario-test/stop");

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-2">Scenario Test Container</p>
        <div className="flex items-center gap-3">
          <span
            className={`w-3 h-3 rounded-full ${
              scenarioState === "running" ? "bg-green-500" : "bg-gray-500"
            }`}
          />
          <span className="text-lg font-semibold capitalize">
            {scenarioState}
          </span>
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Pose Input</p>
        <div className="grid grid-cols-2 gap-3 md:grid-cols-4">
          <label className="text-sm text-gray-300">
            <span className="block text-xs text-gray-400 mb-1">x</span>
            <input
              type="number"
              step="0.1"
              value={pose.x}
              onChange={(e) => updatePose("x", Number(e.target.value))}
              className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </label>
          <label className="text-sm text-gray-300">
            <span className="block text-xs text-gray-400 mb-1">y</span>
            <input
              type="number"
              step="0.1"
              value={pose.y}
              onChange={(e) => updatePose("y", Number(e.target.value))}
              className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </label>
          <label className="text-sm text-gray-300">
            <span className="block text-xs text-gray-400 mb-1">z</span>
            <input
              type="number"
              step="0.01"
              value={pose.z}
              onChange={(e) => updatePose("z", Number(e.target.value))}
              className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </label>
          <label className="text-sm text-gray-300">
            <span className="block text-xs text-gray-400 mb-1">yaw(rad)</span>
            <input
              type="number"
              step="0.1"
              value={pose.yaw}
              onChange={(e) => updatePose("yaw", Number(e.target.value))}
              className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </label>
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Simulation Control</p>
        <div className="flex flex-wrap gap-3">
          <button
            onClick={handleResetRobotPose}
            disabled={loading}
            className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Reset Robot Pose
          </button>
          <button
            onClick={handleResetAmclPose}
            disabled={loading}
            className="bg-indigo-600 hover:bg-indigo-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Reset AMCL Pose
          </button>
          <button
            onClick={handleStartScenario}
            disabled={loading}
            className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Start Scenario
          </button>
          <button
            onClick={handleStopScenario}
            disabled={loading}
            className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Stop Scenario
          </button>
        </div>
        {error && <p className="text-red-400 text-sm">{error}</p>}
      </section>
      <ApiLogPanel logs={sysManager.logs} />
    </div>
  );
}
