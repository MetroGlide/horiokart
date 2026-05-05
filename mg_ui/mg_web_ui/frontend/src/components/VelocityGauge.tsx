import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useTopicSubscriber } from "../hooks/useTopicSubscriber";
import { useTeleop } from "../contexts/TeleopContext";
import { OdomMsg, TwistMsg } from "../types";
import { TOPICS } from "../ros/interfaces";

interface VelocityBarProps {
  label: string;
  cmdValue: number;
  actualValue: number | null;
  maxValue: number;
  unit: string;
}

function VelocityBar({
  label,
  cmdValue,
  actualValue,
  maxValue,
  unit,
}: VelocityBarProps) {
  const clamp = (v: number) => Math.min(Math.abs(v) / maxValue, 1);
  const cmdPct = clamp(cmdValue) * 100;
  const actualPct = actualValue !== null ? clamp(actualValue) * 100 : null;

  return (
    <div>
      <div className="flex justify-between items-baseline mb-0.5">
        <span className="text-xs text-gray-400">{label}</span>
        <div className="flex gap-2 text-xs font-mono">
          <span className="text-blue-400">{cmdValue.toFixed(2)}</span>
          {actualValue !== null && (
            <span className="text-green-400">{actualValue.toFixed(2)}</span>
          )}
          <span className="text-gray-500">{unit}</span>
        </div>
      </div>
      <div className="relative h-2 bg-gray-700 rounded overflow-hidden">
        <div
          className="absolute inset-y-0 left-0 bg-blue-500/60 rounded transition-all duration-75"
          style={{ width: `${cmdPct}%` }}
        />
        {actualPct !== null && (
          <div
            className="absolute inset-y-0 left-0 bg-green-400/80 rounded transition-all duration-150"
            style={{ width: `${actualPct}%`, height: "2px", top: "5px" }}
          />
        )}
      </div>
    </div>
  );
}

interface VelocityGaugeProps {
  client: FoxgloveClientHandle;
  compact?: boolean;
}

export default function VelocityGauge({
  client,
  compact = false,
}: VelocityGaugeProps) {
  const { effectiveGaugeMaxLinear, effectiveGaugeMaxAngular } = useTeleop();
  const cmdVel = useTopicSubscriber<TwistMsg>(
    client,
    TOPICS.CMD_VEL,
    "geometry_msgs/msg/Twist",
  );
  const odom = useTopicSubscriber<OdomMsg>(
    client,
    TOPICS.ODOM,
    "nav_msgs/msg/Odometry",
  );
  const cmdLinear = cmdVel?.linear.x ?? 0;
  const cmdAngular = cmdVel?.angular.z ?? 0;
  const actualLinear = odom?.twist.twist.linear.x ?? null;
  const actualAngular = odom?.twist.twist.angular.z ?? null;

  return (
    <div
      className={`bg-gray-900/85 border border-gray-600/50 rounded-lg backdrop-blur-sm ${compact ? "p-2 space-y-1.5" : "p-3 space-y-2"}`}
    >
      {!compact && (
        <div className="flex items-center gap-2 mb-1">
          <p className="text-xs font-medium text-gray-400">Velocity</p>
          <div className="flex gap-2 text-xs">
            <span className="text-blue-400">■ cmd</span>
            <span className="text-green-400">■ actual</span>
          </div>
        </div>
      )}
      <VelocityBar
        label="Linear"
        cmdValue={cmdLinear}
        actualValue={actualLinear}
        maxValue={effectiveGaugeMaxLinear}
        unit="m/s"
      />
      <VelocityBar
        label="Angular"
        cmdValue={cmdAngular}
        actualValue={actualAngular}
        maxValue={effectiveGaugeMaxAngular}
        unit="rad/s"
      />
    </div>
  );
}
