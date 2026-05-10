import { DiagnosticStatus, DIAG_LEVEL, DIAG_COLOR } from "../../types/ros";

interface DiagnosticsTableProps {
  statuses: DiagnosticStatus[];
  compact?: boolean;
}

export default function DiagnosticsTable({
  statuses,
  compact = false,
}: DiagnosticsTableProps) {
  if (statuses.length === 0) {
    return <p className="text-sm text-gray-500">waiting for /diagnostics…</p>;
  }

  if (compact) {
    return (
      <ul className="space-y-1">
        {statuses.map((s) => (
          <li key={s.name} className="flex gap-2 text-sm">
            <span className={`font-semibold ${DIAG_COLOR[s.level]}`}>
              {DIAG_LEVEL[s.level]}
            </span>
            <span className="text-gray-300">{s.name}</span>
            <span className="text-gray-500">{s.message}</span>
          </li>
        ))}
      </ul>
    );
  }

  return (
    <table className="w-full text-sm">
      <thead>
        <tr className="text-gray-400 text-left border-b border-gray-700">
          <th className="pb-2 w-16">Level</th>
          <th className="pb-2">Name</th>
          <th className="pb-2">Message</th>
        </tr>
      </thead>
      <tbody>
        {statuses.map((s) => (
          <tr key={s.name} className="border-b border-gray-700/50">
            <td className={`py-2 font-semibold ${DIAG_COLOR[s.level]}`}>
              {DIAG_LEVEL[s.level]}
            </td>
            <td className="py-2 text-gray-300">{s.name}</td>
            <td className="py-2 text-gray-500">{s.message}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}
