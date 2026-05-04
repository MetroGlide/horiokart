import { ApiLog } from "../hooks/useSystemManagerClient";

export default function ApiLogPanel({ logs }: { logs: ApiLog[] }) {
  if (logs.length === 0) return null;

  return (
    <section className="bg-gray-800 rounded-lg p-4 space-y-2">
      <p className="text-xs text-gray-400">API Log</p>
      <ul className="space-y-1 max-h-48 overflow-y-auto font-mono text-xs">
        {logs.map((log) => (
          <li key={log.id} className="flex gap-2 items-start">
            <span className="text-gray-500 shrink-0">{log.timestamp}</span>
            <span
              className={`shrink-0 font-bold ${log.success ? "text-green-400" : "text-red-400"}`}
            >
              {log.success ? "OK" : "ERR"}
            </span>
            <span className="text-gray-300 shrink-0">{log.path}</span>
            {log.message && (
              <span className="text-gray-500 break-all">{log.message}</span>
            )}
          </li>
        ))}
      </ul>
    </section>
  );
}
