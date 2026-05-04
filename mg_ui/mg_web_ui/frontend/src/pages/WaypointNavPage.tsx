import { useState } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useServiceCaller } from "../hooks/useServiceCaller";
import { SequencerStatus } from "../types";
import { TOPICS, SERVICES } from "../ros/interfaces";

const STATE_COLOR: Record<string, string> = {
  IDLE: "text-gray-300",
  ON_STARTING: "text-yellow-400",
  NAVIGATING: "text-blue-400",
  ON_ARRIVING: "text-cyan-400",
  GOAL_REACHED: "text-green-400",
  SUSPENDED: "text-yellow-500",
  ERROR: "text-red-400",
};

export default function WaypointNavPage({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const [countdownMs, setCountdownMs] = useState(3000);
  const [jumpIndex, setJumpIndex] = useState(0);
  const { call, loading, error } = useServiceCaller(client);

  const status = useTopicSubscriber<SequencerStatus>(
    client,
    TOPICS.WAYPOINT_STATUS,
    "mg_msgs/msg/SequencerStatus",
  );

  const handleStart = () =>
    call(SERVICES.WAYPOINT_START, { countdown_ms: countdownMs });

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

  const stateColor = status
    ? (STATE_COLOR[status.state] ?? "text-white")
    : "text-gray-500";

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
    </div>
  );
}
