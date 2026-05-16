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

interface ServiceItem {
  key: string;
  label: string;
}

interface ServiceLayer {
  id: string;
  label: string;
  services: readonly ServiceItem[];
}

const SERVICE_LAYERS: ServiceLayer[] = [
  {
    id: "core",
    label: "Core",
    services: [
      { key: "foxglove-bridge", label: "Foxglove Bridge" },
      { key: "diagnostics", label: "Diagnostics" },
    ],
  },
  {
    id: "function",
    label: "Function",
    services: [
      { key: "navigation", label: "Navigation" },
      { key: "slam", label: "SLAM" },
    ],
  },
  {
    id: "tool",
    label: "Tool",
    services: [
      { key: "waypoint-editor", label: "Waypoint Editor" },
      { key: "gazebo-simulation", label: "Gazebo Simulation" },
      { key: "rviz2", label: "RViz2" },
      { key: "rviz2-navigation", label: "RViz2 Navigation" },
      { key: "rviz2-slam", label: "RViz2 SLAM" },
    ],
  },
];

const ALL_SERVICES: readonly ServiceItem[] = SERVICE_LAYERS.flatMap(
  (l) => l.services,
);

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

  const bulkSetLogSet = (
    services: readonly ServiceItem[],
    setState: React.Dispatch<React.SetStateAction<Set<string>>>,
    storageKey: string,
    value: boolean,
  ) => {
    setState((prev) => {
      const next = new Set(prev);
      for (const { key } of services) {
        if (value) {
          next.add(key);
        } else {
          next.delete(key);
        }
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

  const callBulkStop = async (services: readonly ServiceItem[]) => {
    setLoading(true);
    setError(null);
    try {
      for (const { key } of services) {
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

  const callBulkRestartRunning = async (services: readonly ServiceItem[]) => {
    setLoading(true);
    setError(null);
    try {
      const running = services.filter(
        ({ key }) => containers[key] === "running",
      );
      for (const { key } of running) {
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

  const hasRunning = (services: readonly ServiceItem[]) =>
    services.some(({ key }) => containers[key] === "running");

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

  const [openLayers, setOpenLayers] = useState<Set<string>>(
    () => new Set(SERVICE_LAYERS.map((l) => l.id)),
  );

  const toggleLayer = (id: string) => {
    setOpenLayers((prev) => {
      const next = new Set(prev);
      if (next.has(id)) next.delete(id);
      else next.add(id);
      return next;
    });
  };

  return (
    <div className="space-y-2">
      <div className="bg-gray-800 rounded-lg overflow-hidden">
        {accordionHeader("services", "Services")}
        {openSections.has("services") && (
          <div className="px-4 pb-4 pt-1 space-y-3">
            {/* 全体バルク操作 */}
            <div className="flex flex-wrap gap-2">
              <ActionButton
                label="Stop All"
                onClick={() => callBulkStop(ALL_SERVICES)}
                variant="red"
                size="sm"
                disabled={loading}
              />
              <ActionButton
                label="Restart Running"
                onClick={() => callBulkRestartRunning(ALL_SERVICES)}
                variant="blue"
                size="sm"
                disabled={loading || !hasRunning(ALL_SERVICES)}
              />
              <ActionButton
                label="全受信 ON"
                onClick={() =>
                  bulkSetLogSet(
                    ALL_SERVICES,
                    setLogReceiveSet,
                    LOG_RECEIVE_KEY,
                    true,
                  )
                }
                variant="gray"
                size="sm"
                disabled={loading}
              />
              <ActionButton
                label="全受信 OFF"
                onClick={() =>
                  bulkSetLogSet(
                    ALL_SERVICES,
                    setLogReceiveSet,
                    LOG_RECEIVE_KEY,
                    false,
                  )
                }
                variant="gray"
                size="sm"
                disabled={loading}
              />
              <ActionButton
                label="全表示 ON"
                onClick={() =>
                  bulkSetLogSet(
                    ALL_SERVICES,
                    setLogDisplaySet,
                    LOG_DISPLAY_KEY,
                    true,
                  )
                }
                variant="gray"
                size="sm"
                disabled={loading}
              />
              <ActionButton
                label="全表示 OFF"
                onClick={() =>
                  bulkSetLogSet(
                    ALL_SERVICES,
                    setLogDisplaySet,
                    LOG_DISPLAY_KEY,
                    false,
                  )
                }
                variant="gray"
                size="sm"
                disabled={loading}
              />
            </div>

            {/* レイヤーごとのサービス表示 */}
            <div className="space-y-2">
              {SERVICE_LAYERS.map((layer) => (
                <div
                  key={layer.id}
                  className="border border-gray-700 rounded-lg overflow-hidden"
                >
                  {/* レイヤーヘッダー */}
                  <div className="flex items-center bg-gray-750 px-3 py-1.5">
                    <button
                      onClick={() => toggleLayer(layer.id)}
                      className="flex items-center gap-2 flex-1 text-left text-xs font-semibold text-gray-400 hover:text-gray-200 transition-colors"
                    >
                      <svg
                        className={`w-3 h-3 flex-shrink-0 transform transition-transform ${
                          openLayers.has(layer.id) ? "rotate-90" : ""
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
                      {layer.label}
                    </button>
                    <div className="flex gap-1.5 flex-shrink-0">
                      <ActionButton
                        label="Stop All"
                        onClick={() => callBulkStop(layer.services)}
                        variant="red"
                        size="sm"
                        disabled={loading}
                      />
                      <ActionButton
                        label="Restart Running"
                        onClick={() => callBulkRestartRunning(layer.services)}
                        variant="blue"
                        size="sm"
                        disabled={loading || !hasRunning(layer.services)}
                      />
                      <ActionButton
                        label="受信 ON"
                        onClick={() =>
                          bulkSetLogSet(
                            layer.services,
                            setLogReceiveSet,
                            LOG_RECEIVE_KEY,
                            true,
                          )
                        }
                        variant="gray"
                        size="sm"
                        disabled={loading}
                      />
                      <ActionButton
                        label="受信 OFF"
                        onClick={() =>
                          bulkSetLogSet(
                            layer.services,
                            setLogReceiveSet,
                            LOG_RECEIVE_KEY,
                            false,
                          )
                        }
                        variant="gray"
                        size="sm"
                        disabled={loading}
                      />
                      <ActionButton
                        label="表示 ON"
                        onClick={() =>
                          bulkSetLogSet(
                            layer.services,
                            setLogDisplaySet,
                            LOG_DISPLAY_KEY,
                            true,
                          )
                        }
                        variant="gray"
                        size="sm"
                        disabled={loading}
                      />
                      <ActionButton
                        label="表示 OFF"
                        onClick={() =>
                          bulkSetLogSet(
                            layer.services,
                            setLogDisplaySet,
                            LOG_DISPLAY_KEY,
                            false,
                          )
                        }
                        variant="gray"
                        size="sm"
                        disabled={loading}
                      />
                    </div>
                  </div>

                  {/* レイヤー内サービス一覧 */}
                  {openLayers.has(layer.id) && (
                    <div className="px-3 py-2 space-y-2">
                      {layer.services.map(({ key, label }) => (
                        <div
                          key={key}
                          className="flex items-center gap-3 flex-wrap"
                        >
                          <span className="w-36 text-sm text-gray-300">
                            {label}
                          </span>
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
                    </div>
                  )}
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
