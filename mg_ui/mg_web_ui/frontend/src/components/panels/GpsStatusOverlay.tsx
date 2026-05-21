import { NavSatFix } from "../../types/ros";

const STATUS_LABEL: Record<number, string> = {
  [-1]: "NO FIX",
  0: "FIX",
  1: "SBAS",
  2: "GBAS",
};

const STATUS_COLOR: Record<number, string> = {
  [-1]: "text-red-500",
  0: "text-emerald-400",
  1: "text-cyan-400",
  2: "text-cyan-400",
};

interface GpsStatusOverlayProps {
  fix: NavSatFix | null;
}

export default function GpsStatusOverlay({ fix }: GpsStatusOverlayProps) {
  const status = fix?.status.status ?? -1;
  const statusLabel = STATUS_LABEL[status] ?? "NO FIX";
  const statusColor = STATUS_COLOR[status] ?? "text-red-500";

  const lat =
    fix !== null
      ? `${Math.abs(fix.latitude).toFixed(6)}°${fix.latitude >= 0 ? "N" : "S"}`
      : "—";
  const lon =
    fix !== null
      ? `${Math.abs(fix.longitude).toFixed(6)}°${fix.longitude >= 0 ? "E" : "W"}`
      : "—";
  const alt = fix !== null ? `${fix.altitude.toFixed(1)} m` : "—";

  let acc = "—";
  if (
    fix !== null &&
    fix.position_covariance_type !== 0 &&
    fix.position_covariance.length >= 1
  ) {
    acc = `±${Math.sqrt(fix.position_covariance[0]).toFixed(1)} m`;
  }

  return (
    <div className="bg-gray-900/85 border border-gray-600/50 rounded-lg backdrop-blur-sm p-2 space-y-1 w-40">
      <div className="flex items-center justify-between">
        <span className="text-xs font-medium text-gray-400">GPS</span>
        <span className={`text-xs font-mono font-bold ${statusColor}`}>
          {statusLabel}
        </span>
      </div>
      <div className="flex justify-between items-baseline">
        <span className="text-xs text-gray-500">Lat</span>
        <span className="text-xs font-mono text-gray-300">{lat}</span>
      </div>
      <div className="flex justify-between items-baseline">
        <span className="text-xs text-gray-500">Lon</span>
        <span className="text-xs font-mono text-gray-300">{lon}</span>
      </div>
      <div className="flex justify-between items-baseline">
        <span className="text-xs text-gray-500">Alt</span>
        <span className="text-xs font-mono text-gray-300">{alt}</span>
      </div>
      <div className="flex justify-between items-baseline">
        <span className="text-xs text-gray-500">Acc</span>
        <span className="text-xs font-mono text-gray-300">{acc}</span>
      </div>
    </div>
  );
}
