import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useServiceCaller } from "../hooks/useServiceCaller";
import { useNav2Status } from "../hooks/useNav2Status";
import { useSimulation } from "../contexts/SimulationContext";
import { SequencerStatus, GOAL_STATUS, GOAL_STATUS_COLOR } from "../types";
import { TOPICS, SERVICES } from "../ros/interfaces";
import ApiLogPanel from "../components/ApiLogPanel";

const STATE_COLOR: Record<string, string> = {
  IDLE: "text-gray-300",
  ON_STARTING: "text-yellow-400",
  NAVIGATING: "text-blue-400",
  ON_ARRIVING: "text-cyan-400",
  GOAL_REACHED: "text-green-400",
  SUSPENDED: "text-yellow-500",
  ERROR: "text-red-400",
};

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
    header: { stamp: { sec, nanosec }, frame_id: "map" },
    pose: {
      pose: {
        position: { x: pose.x, y: pose.y, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
      },
      covariance,
    },
  };
}

export default function WaypointNavPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [countdownMs, setCountdownMs] = useState(3000);
  const [jumpIndex, setJumpIndex] = useState(0);
  const { call, loading, error } = useServiceCaller(client);
  const { isSimulation } = useSimulation();

  const status = useTopicSubscriber<SequencerStatus>(
    client,
    TOPICS.WAYPOINT_STATUS,
    "mg_msgs/msg/SequencerStatus",
  );

  const nav2 = useNav2Status(client);

  const [simLoading, setSimLoading] = useState(false);
  const [simError, setSimError] = useState<string | null>(null);
  const [pose, setPose] = useState<PoseInput>({ x: 0.0, y: 0.0, z: 0.05, yaw: 0.0 });
  const { containers, callApi } = sysManager;
  const scenarioState = containers["scenario-test"] ?? "unknown";

  const updatePose = (key: keyof PoseInput, value: number) =>
    setPose((prev) => ({ ...prev, [key]: value }));

  const callSim = async (path: string, body?: unknown) => {
    setSimLoading(true);
    setSimError(null);
    try {
      const result = await callApi(path, body);
      if (!result.success) setSimError(result.message);
    } catch (e) {
      setSimError(e instanceof Error ? e.message : String(e));
    } finally {
      setSimLoading(false);
    }
  };

  const handleStart = () => call(SERVICES.WAYPOINT_START, { countdown_ms: countdownMs });
  const handleStop = () => call(SERVICES.WAYPOINT_STOP, {});
  const handlePause = () =>
    client.publish(TOPICS.WAYPOINT_PAUSE_REQUEST, "mg_msgs/msg/PauseRequest", {
      requester_id: "web_ui",
      active: true,
      heartbeat_period_s: 0.0,
      reason: "manual pause",
    });
  const handleResume = () =>
    client.publish(TOPICS.WAYPOINT_PAUSE_REQUEST, "mg_msgs/msg/PauseRequest", {
      requester_id: "web_ui",
      active: false,
      heartbeat_period_s: 0.0,
      reason: "",
    });
  const handleJump = () =>
    client.publish(TOPICS.WAYPOINT_SET_NEXT_INDEX, "std_msgs/msg/Int16", { data: jumpIndex });
  const handleReload = () => call(SERVICES.WAYPOINT_RELOAD, {});

  const handleResetRobotPose = () => callSim("/simulation/reset-pose", pose);
  const handleResetAmclPose = () => {
    setSimError(null);
    try {
      client.publish(
        TOPICS.INITIALPOSE,
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        buildInitialPoseMessage(pose),
      );
    } catch (e) {
      setSimError(e instanceof Error ? e.message : String(e));
    }
  };
  const handleStartScenario = () => callSim("/scenario-test/start");
  const handleStopScenario = () => callSim("/scenario-test/stop");

  const stateColor = status ? (STATE_COLOR[status.state] ?? "text-white") : "text-gray-500";
  const actionStatusLabel =
    nav2.actionStatus !== null ? (GOAL_STATUS[nav2.actionStatus] ?? String(nav2.actionStatus)) : "—";
  const actionStatusColor =
    nav2.actionStatus !== null ? (GOAL_STATUS_COLOR[nav2.actionStatus] ?? "text-white") : "text-gray-500";

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-1">State</p>
        <p className={`text-3xl font-bold ${stateColor}`}>
          {status?.state ?? "—"}
        </p>
        {status && (
          <div className="mt-2 grid grid-cols-2 gap-2 text-sm text-gray-300">
            <span>
              Waypoint:{" "}
              {Math.min(status.current_index + 1, status.total_waypoints)} /{" "}
              {status.total_waypoints}
            </span>
            <span>Remaining: {status.distance_remaining.toFixed(1)} m</span>
            {status.countdown_ms_remaining > 0 && (
              <span className="col-span-2 text-yellow-400">
                Countdown: {(status.countdown_ms_remaining / 1000).toFixed(1)} s
              </span>
            )}
            {status.is_paused && (
              <span className="col-span-2 text-yellow-500">
                Paused by: {status.pause_requesters.join(", ")}
              </span>
            )}
          </div>
        )}
      </section>

      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-3">Nav2 Status</p>
        <div className="grid grid-cols-2 gap-3 text-sm">
          <div>
            <span className="text-xs text-gray-400">Navigation Lifecycle</span>
            <p className={`font-semibold ${nav2.navLifecycleActive ? "text-green-400" : "text-red-400"}`}>
              {nav2.navLifecycleActive ? "Active" : "Inactive"}
            </p>
          </div>
          <div>
            <span className="text-xs text-gray-400">Localization Lifecycle</span>
            <p className={`font-semibold ${nav2.locLifecycleActive ? "text-green-400" : "text-red-400"}`}>
              {nav2.locLifecycleActive ? "Active" : "Inactive"}
            </p>
          </div>
          <div>
            <span className="text-xs text-gray-400">Action Status</span>
            <p className={`font-semibold ${actionStatusColor}`}>{actionStatusLabel}</p>
          </div>
          <div>
            <span className="text-xs text-gray-400">AMCL</span>
            <p className={`font-semibold ${nav2.amclActive ? "text-green-400" : "text-gray-500"}`}>
              {nav2.amclActive ? "Receiving" : "No data"}
            </p>
          </div>
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Control</p>
        <div className="flex flex-wrap gap-3 items-end">
          <div>
            <label className="block text-xs text-gray-400 mb-1">
              Countdown (ms)
            </label>
            <input
              type="number"
              min={0}
              step={500}
              value={countdownMs}
              onChange={(e) => setCountdownMs(Number(e.target.value))}
              className="w-28 bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </div>
          <button
            onClick={handleStart}
            disabled={loading}
            className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            START
          </button>
          <button
            onClick={handleStop}
            disabled={loading}
            className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            STOP
          </button>
          <button
            onClick={handlePause}
            disabled={loading}
            className="bg-yellow-600 hover:bg-yellow-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            PAUSE
          </button>
          <button
            onClick={handleResume}
            disabled={loading}
            className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            RESUME
          </button>
          <button
            onClick={handleReload}
            disabled={loading}
            className="bg-gray-600 hover:bg-gray-500 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Reload WPs
          </button>
        </div>

        <div className="flex gap-3 items-end">
          <div>
            <label className="block text-xs text-gray-400 mb-1">
              Jump to Index
            </label>
            <input
              type="number"
              min={0}
              value={jumpIndex}
              onChange={(e) => setJumpIndex(Number(e.target.value))}
              className="w-20 bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </div>
          <button
            onClick={handleJump}
            className="bg-gray-600 hover:bg-gray-500 px-4 py-2 rounded font-medium text-sm"
          >
            Jump
          </button>
        </div>

        {error && <p className="text-red-400 text-sm">{error}</p>}
      </section>

      {isSimulation && (
        <section className="bg-gray-800 rounded-lg p-4 space-y-3">
          <p className="text-xs text-gray-400">Simulation</p>
          <div className="flex items-center gap-3">
            <span
              className={`w-3 h-3 rounded-full ${
                scenarioState === "running" ? "bg-green-500" : "bg-gray-500"
              }`}
            />
            <span className="text-sm text-gray-300">Scenario: {scenarioState}</span>
          </div>
          <div className="grid grid-cols-2 gap-3 md:grid-cols-4">
            {(["x", "y", "z", "yaw"] as const).map((k) => (
              <label key={k} className="text-sm text-gray-300">
                <span className="block text-xs text-gray-400 mb-1">
                  {k === "yaw" ? "yaw(rad)" : k}
                </span>
                <input
                  type="number"
                  step={k === "z" ? "0.01" : "0.1"}
                  value={pose[k]}
                  onChange={(e) => updatePose(k, Number(e.target.value))}
                  className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
                />
              </label>
            ))}
          </div>
          <div className="flex flex-wrap gap-3">
            <button
              onClick={handleResetRobotPose}
              disabled={simLoading}
              className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Reset Robot Pose
            </button>
            <button
              onClick={handleResetAmclPose}
              disabled={simLoading}
              className="bg-indigo-600 hover:bg-indigo-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Reset AMCL Pose
            </button>
            <button
              onClick={handleStartScenario}
              disabled={simLoading}
              className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Start Scenario
            </button>
            <button
              onClick={handleStopScenario}
              disabled={simLoading}
              className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Stop Scenario
            </button>
          </div>
          {simError && <p className="text-red-400 text-sm">{simError}</p>}
        </section>
      )}

      <ApiLogPanel logs={sysManager.logs} />
    </div>
  );
}
