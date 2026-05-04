import { useState } from "react"
import { useDiagnosticsMap } from "../hooks/useDiagnosticsMap"
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient"
import { SystemManagerHandle } from "../hooks/useSystemManagerClient"
import { DIAG_COLOR, DIAG_LEVEL } from "../types"
import ApiLogPanel from "../components/ApiLogPanel"

export default function SystemPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle
  sysManager: SystemManagerHandle
}) {
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const { callApi, containers } = sysManager
  const diagStatuses = useDiagnosticsMap(client)

  const SERVICES = [
    { key: "foxglove-bridge", label: "Foxglove Bridge" },
    { key: "diagnostics", label: "Diagnostics" },
    { key: "waypoint-editor", label: "Waypoint Editor" },
  ] as const

  const handleServiceStart = async (service: string) => {
    setLoading(true)
    setError(null)
    try {
      const result = await callApi(`/${service}/start`)
      if (!result.success) setError(result.message)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setLoading(false)
    }
  }

  const handleServiceStop = async (service: string) => {
    setLoading(true)
    setError(null)
    try {
      const result = await callApi(`/${service}/stop`)
      if (!result.success) setError(result.message)
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setLoading(false)
    }
  }

  const statusColor = (status: string | undefined) => {
    if (status === "running") return "text-green-400"
    if (status === "exited" || status === "dead") return "text-red-400"
    return "text-gray-400"
  }

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4 space-y-3">
        <p className="text-xs text-gray-400">Services</p>
        {error && <p className="text-red-400 text-sm">{error}</p>}
        <div className="space-y-2">
          {SERVICES.map(({ key, label }) => (
            <div key={key} className="flex items-center gap-3">
              <span className="w-36 text-sm text-gray-300">{label}</span>
              <span className={`w-20 text-xs font-mono ${statusColor(containers[key])}`}>
                {containers[key] ?? "unknown"}
              </span>
              <button
                onClick={() => handleServiceStart(key)}
                disabled={loading || containers[key] === "running"}
                className="bg-green-700 hover:bg-green-600 disabled:opacity-40 px-3 py-1 rounded text-xs font-medium"
              >
                Start
              </button>
              <button
                onClick={() => handleServiceStop(key)}
                disabled={loading || containers[key] !== "running"}
                className="bg-red-700 hover:bg-red-600 disabled:opacity-40 px-3 py-1 rounded text-xs font-medium"
              >
                Stop
              </button>
            </div>
          ))}
        </div>
      </section>

      <section className="bg-gray-800 rounded-lg p-4">
        <p className="text-xs text-gray-400 mb-3">Diagnostics Detail</p>
        {diagStatuses.length > 0 ? (
          <table className="w-full text-sm">
            <thead>
              <tr className="text-gray-400 text-left border-b border-gray-700">
                <th className="pb-2 w-16">Level</th>
                <th className="pb-2">Name</th>
                <th className="pb-2">Message</th>
              </tr>
            </thead>
            <tbody>
              {diagStatuses.map((s) => (
                <tr key={s.name} className="border-b border-gray-700/50">
                  <td className={`py-2 font-semibold ${DIAG_COLOR[s.level]}`}>{DIAG_LEVEL[s.level]}</td>
                  <td className="py-2 text-gray-300">{s.name}</td>
                  <td className="py-2 text-gray-500">{s.message}</td>
                </tr>
              ))}
            </tbody>
          </table>
        ) : (
          <p className="text-sm text-gray-500">waiting for /diagnostics…</p>
        )}
      </section>
      <ApiLogPanel logs={sysManager.logs} />
    </div>
  )
}
