import { useMemo, useState } from "react";
import { useDiagnosticsMap } from "../hooks/useDiagnosticsMap";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { useDockerLogStream } from "../hooks/useDockerLogStream";
import DiagnosticsTable from "../components/panels/DiagnosticsTable";
import ApiLogPanel from "../components/panels/ApiLogPanel";
import ServiceLogPanel from "../components/panels/ServiceLogPanel";
import ActionButton from "../components/ui/ActionButton";
import StatusBadge from "../components/ui/StatusBadge";

const LOG_RECEIVE_KEY = "mg_ui_log_receive";
const LOG_DISPLAY_KEY = "mg_ui_log_display";

function loadServiceSet(storageKey: string): Set<string> {
  try {
    const raw = localStorage.getItem(storageKey);
    if (!raw) return new Set();
    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) return new Set();
    return new Set(
      parsed.filter((item): item is string => typeof item === "string"),
    );
  } catch {
    return new Set();
  }
}

function persistServiceSet(storageKey: string, value: Set<string>): void {
  localStorage.setItem(storageKey, JSON.stringify([...value]));
}

export default function SystemPage({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [logReceiveSet, setLogReceiveSet] = useState<Set<string>>(() =>
    loadServiceSet(LOG_RECEIVE_KEY),
  );
  const [logDisplaySet, setLogDisplaySet] = useState<Set<string>>(() =>
    loadServiceSet(LOG_DISPLAY_KEY),
  );
  const { callApi, containers } = sysManager;
  const diagStatuses = useDiagnosticsMap(client);

  const MANAGED_SERVICES = [
    { key: "navigation", label: "Navigation" },
    { key: "slam", label: "SLAM" },
    { key: "foxglove-bridge", label: "Foxglove Bridge" },
    { key: "diagnostics", label: "Diagnostics" },
    { key: "waypoint-editor", label: "Waypoint Editor" },
    { key: "gazebo-simulation", label: "Gazebo Simulation" },
    { key: "rviz2", label: "RViz2" },
    { key: "rviz2-navigation", label: "RViz2 Navigation" },
    { key: "rviz2-slam", label: "RViz2 SLAM" },
  ] as const;

  const subscribedServices = useMemo(
    () => [...logReceiveSet].sort(),
    [logReceiveSet],
  );
  const { entries, connected, clear } = useDockerLogStream(subscribedServices);

  const toggleServiceSet = (
    key: string,
    setState: React.Dispatch<React.SetStateAction<Set<string>>>,
    storageKey: string,
  ) => {
    setState((prev) => {
      const next = new Set(prev);
      if (next.has(key)) {
        next.delete(key);
      } else {
        next.add(key);
      }
      persistServiceSet(storageKey, next);
      return next;
    });
  };

  const callService = async (path: string) => {
    setLoading(true);
    setError(null);
    try {
      const result = await callApi(path);
      if (!result.success) setError(result.message);
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  };

  const callBulkStop = async () => {
    setLoading(true);
    setError(null);
    try {
      for (const { key } of MANAGED_SERVICES) {
        const result = await callApi(`/${key}/stop`);
        if (!result.success) {
          setError(result.message);
          break;
        }
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  };

  const callBulkRestartRunning = async () => {
    setLoading(true);
    setError(null);
    try {
      const runningServices = MANAGED_SERVICES.filter(
        ({ key }) => containers[key] === "running",
      );
      for (const { key } of runningServices) {
        const result = await callApi(`/${key}/restart`);
        if (!result.success) {
          setError(result.message);
          break;
        }
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : String(e));
    } finally {
      setLoading(false);
    }
  };

  const hasRunningService = MANAGED_SERVICES.some(
    ({ key }) => containers[key] === "running",
  );

  const [openSections, setOpenSections] = useState<Set<string>>(
    () => new Set(["services", "diagnostics", "service-log", "api-log"]),
  );

  const toggleSection = (id: string) => {
    setOpenSections((prev) => {
      const next = new Set(prev);
      if (next.has(id)) next.delete(id);
      else next.add(id);
      return next;
    });
  };

  const chevron = (id: string) => (
    <svg
      className={`w-4 h-4 flex-shrink-0 transform transition-transform ${
        openSections.has(id) ? "rotate-90" : ""
      }`}
      fill="none"
      viewBox="0 0 24 24"
      stroke="currentColor"
    >
      <path
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeWidth={2}
        d="M9 5l7 7-7 7"
      />
    </svg>
  );

  const accordionHeader = (id: string, label: string) => (
    <button
      onClick={() => toggleSection(id)}
      className="w-full flex items-center justify-between px-4 py-2.5 text-sm font-medium text-gray-300 hover:text-white hover:bg-gray-700 transition-colors"
    >
      <span>{label}</span>
      {chevron(id)}
    </button>
  );

  return (
    <div className="space-y-2">
      <div className="bg-gray-800 rounded-lg overflow-hidden">
        {accordionHeader("services", "Services")}
        {openSections.has("services") && (
          <div className="px-4 pb-4 pt-1 space-y-3">
            <div className="flex flex-wrap gap-2">
              <ActionButton
                label="Stop All"
                onClick={callBulkStop}
                variant="red"
                size="sm"
                disabled={loading}
              />
              <ActionButton
                label="Restart Running"
                onClick={callBulkRestartRunning}
                variant="blue"
                size="sm"
                disabled={loading || !hasRunningService}
              />
            </div>
            <div className="space-y-3">
              {MANAGED_SERVICES.map(({ key, label }) => (
                <div key={key} className="flex items-center gap-3 flex-wrap">
                  <span className="w-36 text-sm text-gray-300">{label}</span>
                  <StatusBadge status={containers[key] ?? "unknown"} />
                  <div className="flex gap-2">
                    {containers[key] === "running" ? (
                      <ActionButton
                        label="Restart"
                        onClick={() => callService(`/${key}/restart`)}
                        variant="blue"
                        size="sm"
                        disabled={loading}
                      />
                    ) : (
                      <ActionButton
                        label="Start"
                        onClick={() => callService(`/${key}/start`)}
                        variant="green"
                        size="sm"
                        disabled={loading}
                      />
                    )}
                    <ActionButton
                      label="Stop"
                      onClick={() => callService(`/${key}/stop`)}
                      variant="red"
                      size="sm"
                      disabled={loading}
                    />
                  </div>
                  <div className="flex items-center gap-4 ml-2">
                    <label className="flex items-center gap-1.5 text-xs text-gray-300 select-none cursor-pointer">
                      <input
                        type="checkbox"
                        checked={logReceiveSet.has(key)}
                        onChange={() =>
                          toggleServiceSet(
                            key,
                            setLogReceiveSet,
                            LOG_RECEIVE_KEY,
                          )
                        }
                      />
                      ログ受信
                    </label>
                    <label className="flex items-center gap-1.5 text-xs text-gray-300 select-none cursor-pointer">
                      <input
                        type="checkbox"
                        checked={logDisplaySet.has(key)}
                        onChange={() =>
                          toggleServiceSet(
                            key,
                            setLogDisplaySet,
                            LOG_DISPLAY_KEY,
                          )
                        }
                      />
                      ログ表示
                    </label>
                  </div>
                </div>
              ))}
              {error && <p className="text-red-400 text-sm">{error}</p>}
            </div>
          </div>
        )}
      </div>

      <div className="bg-gray-800 rounded-lg overflow-hidden">
        {accordionHeader("diagnostics", "Diagnostics")}
        {openSections.has("diagnostics") && (
          <div className="px-4 pb-4 pt-1">
            <DiagnosticsTable statuses={diagStatuses} />
          </div>
        )}
      </div>

      <div className="bg-gray-800 rounded-lg overflow-hidden">
        {accordionHeader("service-log", "Service Log")}
        {openSections.has("service-log") && (
          <ServiceLogPanel
            entries={entries}
            displayServices={logDisplaySet}
            connected={connected}
            receivingServices={logReceiveSet}
            onClear={clear}
          />
        )}
      </div>

      <div className="bg-gray-800 rounded-lg overflow-hidden">
        {accordionHeader("api-log", "API Log")}
        {openSections.has("api-log") && <ApiLogPanel logs={sysManager.logs} />}
      </div>
    </div>
  );
}
