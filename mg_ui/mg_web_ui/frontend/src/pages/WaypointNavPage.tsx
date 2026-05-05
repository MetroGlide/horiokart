import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useServiceCaller } from "../hooks/useServiceCaller";
import { useNav2Status } from "../hooks/useNav2Status";
import { useSimulation } from "../contexts/SimulationContext";
import {
  SequencerStatus,
  BoolMsg,
  CollisionDetectorState,
  GOAL_STATUS,
  GOAL_STATUS_COLOR,
} from "../types";
import { TOPICS, SERVICES } from "../ros/interfaces";
import SectionCard from "../components/SectionCard";
import RobotPageLayout from "../components/RobotPageLayout";
import VelocityGauge from "../components/VelocityGauge";
import ApiLogSection from "../components/sections/ApiLogSection";
import ContainerStatusCard from "../components/sections/ContainerStatusCard";
import ServiceControlCard from "../components/sections/ServiceControlCard";
import SimulationPoseSection, {
  PoseInput,
} from "../components/sections/SimulationPoseSection";

const STATE_COLOR: Record<string, string> = {
  IDLE: "text-gray-300",
  ON_STARTING: "text-yellow-400",
  NAVIGATING: "text-blue-400",
  ON_ARRIVING: "text-cyan-400",
  GOAL_REACHED: "text-green-400",
  SUSPENDED: "text-yellow-500",
  ERROR: "text-red-400",
};

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

  const emergencyStop = useTopicSubscriber<BoolMsg>(
    client,
    TOPICS.EMERGENCY_STOP,
    "std_msgs/msg/Bool",
  );

  const collisionState = useTopicSubscriber<CollisionDetectorState>(
    client,
    TOPICS.COLLISION_STATE,
    "nav2_msgs/msg/CollisionDetectorState",
  );

  const [sysLoading, setSysLoading] = useState(false);
  const [sysError, setSysError] = useState<string | null>(null);
  const { containers, callApi } = sysManager;
  const navState = containers["navigation"] ?? "unknown";
  const scenarioState = containers["scenario-test"] ?? "unknown";

  const callSystemManager = async (path: string, body?: unknown) => {
    setSysLoading(true);
    setSysError(null);
    try {
      const result = await callApi(path, body);
      if (!result.success) setSysError(result.message);
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    } finally {
      setSysLoading(false);
    }
  };

  const handleStart = () =>
    call(SERVICES.WAYPOINT_START, { countdown_ms: countdownMs });
  const handleStartImmediate = () =>
    call(SERVICES.WAYPOINT_START, { countdown_ms: 0 });
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
    client.publish(TOPICS.WAYPOINT_SET_NEXT_INDEX, "std_msgs/msg/Int16", {
      data: jumpIndex,
    });
  const handleReload = () => call(SERVICES.WAYPOINT_RELOAD, {});

  const handleResetRobotPose = (pose: PoseInput) =>
    callSystemManager("/simulation/reset-pose", pose);
  const handleResetAmclPose = (pose: PoseInput) => {
    setSysError(null);
    try {
      client.publish(
        TOPICS.INITIALPOSE,
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        buildInitialPoseMessage(pose),
      );
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    }
  };

  const stateColor = status
    ? (STATE_COLOR[status.state] ?? "text-white")
    : "text-gray-500";
  const actionStatusLabel =
    nav2.actionStatus !== null
      ? (GOAL_STATUS[nav2.actionStatus] ?? String(nav2.actionStatus))
      : "—";
  const actionStatusColor =
    nav2.actionStatus !== null
      ? (GOAL_STATUS_COLOR[nav2.actionStatus] ?? "text-white")
      : "text-gray-500";

  const accordionItems = [
    {
      id: "status",
      label: "コンテナステータス",
      children: (
        <div className="space-y-2">
          <ContainerStatusCard title="Navigation Container" status={navState} />
          <ServiceControlCard
            title="Control"
            buttons={[
              {
                label: "Start",
                onClick: () => callSystemManager("/navigation/start"),
                variant: "green",
              },
              {
                label: "Stop",
                onClick: () => callSystemManager("/navigation/stop"),
                variant: "red",
              },
            ]}
            loading={sysLoading}
            error={sysError}
          />
        </div>
      ),
    },
    {
      id: "status",
      label: "WaypointNav ステータス",
      children: (
        <div className="space-y-2">
          <SectionCard title="Waypoint Sequencer State">
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
                    Countdown:{" "}
                    {(status.countdown_ms_remaining / 1000).toFixed(1)} s
                  </span>
                )}
                {status.is_paused && (
                  <span className="col-span-2 text-yellow-500">
                    Paused by: {status.pause_requesters.join(", ")}
                  </span>
                )}
              </div>
            )}
          </SectionCard>
          <SectionCard title="Nav2 Status">
            <div className="grid grid-cols-2 gap-3 text-sm">
              <div>
                <span className="text-xs text-gray-400">
                  Navigation Lifecycle
                </span>
                <p
                  className={`font-semibold ${nav2.navLifecycleActive ? "text-green-400" : "text-red-400"}`}
                >
                  {nav2.navLifecycleActive ? "Active" : "Inactive"}
                </p>
              </div>
              <div>
                <span className="text-xs text-gray-400">
                  Localization Lifecycle
                </span>
                <p
                  className={`font-semibold ${nav2.locLifecycleActive ? "text-green-400" : "text-red-400"}`}
                >
                  {nav2.locLifecycleActive ? "Active" : "Inactive"}
                </p>
              </div>
              <div>
                <span className="text-xs text-gray-400">Action Status</span>
                <p className={`font-semibold ${actionStatusColor}`}>
                  {actionStatusLabel}
                </p>
              </div>
              <div>
                <span className="text-xs text-gray-400">AMCL</span>
                <p
                  className={`font-semibold ${nav2.amclActive ? "text-green-400" : "text-gray-500"}`}
                >
                  {nav2.amclActive ? "Receiving" : "No data"}
                </p>
              </div>
            </div>
          </SectionCard>
          <SectionCard title="Safety">
            <div className="flex flex-wrap gap-2 text-sm">
              <span
                className={`px-2 py-0.5 rounded font-semibold ${
                  emergencyStop?.data
                    ? "bg-red-600 text-white"
                    : "bg-gray-700 text-gray-400"
                }`}
              >
                E-Stop {emergencyStop?.data ? "ACTIVE" : "OFF"}
              </span>
              {collisionState &&
                collisionState.polygons.map((name, i) => (
                  <span
                    key={name}
                    className={`px-2 py-0.5 rounded text-xs font-medium ${
                      collisionState.detections[i]
                        ? "bg-orange-600 text-white"
                        : "bg-gray-700 text-gray-500"
                    }`}
                  >
                    {name}
                  </span>
                ))}
              {!collisionState && (
                <span className="text-xs text-gray-500">
                  collision: no data
                </span>
              )}
            </div>
          </SectionCard>
          <SectionCard title="Velocity">
            <VelocityGauge client={client} />
          </SectionCard>
        </div>
      ),
    },
    {
      id: "control",
      label: "コントロール",
      children: (
        <div className="space-y-2">
          <SectionCard title="ナビゲーション制御">
            <div className="flex flex-wrap gap-3 items-end">
              <button
                onClick={handleStartImmediate}
                disabled={loading}
                className="bg-green-700 hover:bg-green-800 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
              >
                START IMMEDIATE
              </button>
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
                onClick={handleReload}
                disabled={loading}
                className="bg-gray-600 hover:bg-gray-500 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
              >
                Reload WPs
              </button>
            </div>
            {error && <p className="text-red-400 text-sm">{error}</p>}
          </SectionCard>
          <SectionCard title="Waypoint Jump">
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
              <div className="space-y-2">
                <ContainerStatusCard
                  title="シナリオテストコンテナ"
                  status={scenarioState}
                />
                <ServiceControlCard
                  title="シナリオテスト制御"
                  buttons={[
                    {
                      label: "Start",
                      onClick: () => callSystemManager("/scenario-test/start"),
                      variant: "green",
                    },
                    {
                      label: "Stop",
                      onClick: () => callSystemManager("/scenario-test/stop"),
                      variant: "red",
                    },
                  ]}
                  loading={sysLoading}
                />
                <SimulationPoseSection
                  onResetRobot={handleResetRobotPose}
                  onResetAmcl={handleResetAmclPose}
                  loading={sysLoading}
                  error={sysError}
                />
              </div>
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
      defaultOpen={[
        "status",
        "control",
        ...(isSimulation ? ["simulation"] : []),
      ]}
      viewerMode="3d"
    />
  );
}
