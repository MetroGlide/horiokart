import { ConnectionStatus } from "../hooks/useFoxgloveClient";

const CONFIG: Record<ConnectionStatus, { label: string; className: string }> = {
  connected: { label: "Connected", className: "bg-green-600" },
  connecting: {
    label: "Connecting…",
    className: "bg-yellow-600 animate-pulse",
  },
  disconnected: { label: "Disconnected", className: "bg-gray-600" },
  error: { label: "Error", className: "bg-red-600" },
};

export default function ConnectionBadge({
  status,
}: {
  status: ConnectionStatus;
}) {
  const { label, className } = CONFIG[status];
  return (
    <span className={`text-xs font-semibold px-2 py-1 rounded ${className}`}>
      {label}
    </span>
  );
}
