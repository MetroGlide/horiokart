import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../../hooks/useTopicSubscriber";
import { Float32Msg } from "../../types";
import { TOPICS } from "../../ros/interfaces";

interface MetricBarProps {
  label: string;
  value: number | null;
  unit?: string;
  warnThreshold?: number;
  errorThreshold?: number;
}

function MetricBar({
  label,
  value,
  unit = "%",
  warnThreshold = 70,
  errorThreshold = 90,
}: MetricBarProps) {
  const pct = value !== null ? Math.min(Math.max(value, 0), 100) : 0;
  const color =
    value === null
      ? "bg-gray-600"
      : value >= errorThreshold
        ? "bg-red-500"
        : value >= warnThreshold
          ? "bg-yellow-500"
          : "bg-emerald-500";

  return (
    <div>
      <div className="flex justify-between items-baseline mb-0.5">
        <span className="text-xs text-gray-400">{label}</span>
        <span className="text-xs font-mono text-gray-300">
          {value !== null ? `${value.toFixed(1)}${unit}` : "—"}
        </span>
      </div>
      <div className="h-2 bg-gray-700 rounded overflow-hidden">
        <div
          className={`h-full ${color} rounded transition-all duration-300`}
          style={{ width: `${pct}%` }}
        />
      </div>
    </div>
  );
}

interface SystemMetricsProps {
  client: FoxgloveClientHandle;
  compact?: boolean;
}

export default function SystemMetrics({
  client,
  compact = false,
}: SystemMetricsProps) {
  const cpu = useTopicSubscriber<Float32Msg>(
    client,
    TOPICS.CPU_USAGE,
    "std_msgs/msg/Float32",
  );
  const memory = useTopicSubscriber<Float32Msg>(
    client,
    TOPICS.MEMORY_USAGE,
    "std_msgs/msg/Float32",
  );

  return (
    <div
      className={`bg-gray-900/85 border border-gray-600/50 rounded-lg backdrop-blur-sm ${compact ? "p-2 space-y-1.5" : "p-3 space-y-2"}`}
    >
      {!compact && (
        <p className="text-xs font-medium text-gray-400 mb-1">System</p>
      )}
      <MetricBar label="CPU" value={cpu?.data ?? null} />
      <MetricBar label="MEM" value={memory?.data ?? null} />
    </div>
  );
}
