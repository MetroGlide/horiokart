import { useState, useEffect } from "react";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../../hooks/useSystemManagerClient";
import { useServiceCaller } from "../../hooks/useServiceCaller";
import { SERVICES } from "../../ros/services";
import { loadSettings, saveSettings } from "../../utils/settingsApi";
import SectionCard from "../layout/SectionCard";
import ContainerStatusCard from "./ContainerStatusCard";
import ValueConfirmDialog from "../ui/ValueConfirmDialog";

const RATE_OPTIONS = [0.5, 1.0, 1.5, 2.0] as const;

const SETTINGS_KEY = "rosbagReplayInput";

export default function RosbagReplaySection({
  client,
  sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager: SystemManagerHandle;
}) {
  const [file, setFile] = useState("");
  const [topics, setTopics] = useState("");
  const [rate, setRate] = useState(1.0);
  const [isPaused, setIsPaused] = useState(false);
  const [sysLoading, setSysLoading] = useState(false);
  const [sysError, setSysError] = useState<string | null>(null);
  const [dialogOpen, setDialogOpen] = useState(false);
  const [pendingFile, setPendingFile] = useState("");
  const [pendingTopics, setPendingTopics] = useState("");

  const {
    call: callRos,
    loading: rosLoading,
    error: rosError,
  } = useServiceCaller(client);

  const isRunning = sysManager.containers["rosbag-replay"] === "running";

  useEffect(() => {
    loadSettings().then((data) => {
      const s = data[SETTINGS_KEY] as
        | { file?: string; topics?: string }
        | undefined;
      if (s?.file !== undefined) setFile(s.file);
      if (s?.topics !== undefined) setTopics(s.topics);
    });
  }, []);

  const persistInputs = (nextFile: string, nextTopics: string) => {
    saveSettings(SETTINGS_KEY, { file: nextFile, topics: nextTopics });
  };

  const callSys = async (path: string, body?: unknown) => {
    setSysLoading(true);
    setSysError(null);
    try {
      const result = await sysManager.callApi(path, body);
      if (!result.success) setSysError(result.message);
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    } finally {
      setSysLoading(false);
    }
  };

  const handleStart = () => {
    const topicList = topics
      .split(",")
      .map((t) => t.trim())
      .filter((t) => t.length > 0);
    callSys("/rosbag-replay/start", { file, topics: topicList });
    setIsPaused(false);
  };

  const handleStop = () => {
    callSys("/rosbag-replay/stop");
    setIsPaused(false);
  };

  const handlePause = async () => {
    await callRos(SERVICES.ROSBAG_PAUSE, {});
    setIsPaused(true);
  };

  const handleResume = async () => {
    await callRos(SERVICES.ROSBAG_RESUME, {});
    setIsPaused(false);
  };

  const handleSetRate = async (r: number) => {
    await callRos(SERVICES.ROSBAG_SET_RATE, { rate: r });
    setRate(r);
  };

  const handleLoadFromEnv = async () => {
    setSysLoading(true);
    setSysError(null);
    try {
      const BASE_URL = `http://${window.location.hostname}:8001`;
      const res = await fetch(`${BASE_URL}/rosbag-replay/env`);
      const data = await res.json();
      const envFile: string = data.file ?? "";
      const envTopics: string = (data.topics as string[]).join(", ");
      setPendingFile(envFile);
      setPendingTopics(envTopics);
      setDialogOpen(true);
    } catch (e) {
      setSysError(e instanceof Error ? e.message : String(e));
    } finally {
      setSysLoading(false);
    }
  };

  const handleDialogConfirm = () => {
    setFile(pendingFile);
    setTopics(pendingTopics);
    persistInputs(pendingFile, pendingTopics);
    setDialogOpen(false);
  };

  return (
    <div className="space-y-2">
      <ContainerStatusCard
        title="Rosbag Replay"
        status={sysManager.containers["rosbag-replay"] ?? "unknown"}
      />

      <SectionCard title="File">
        <div className="space-y-2">
          <div>
            <label className="text-xs text-gray-400 block mb-1">
              File Path (container absolute path)
            </label>
            <input
              type="text"
              value={file}
              onChange={(e) => setFile(e.target.value)}
              onBlur={() => persistInputs(file, topics)}
              placeholder="/root/ros2_data/example.bag"
              className="w-full bg-gray-700 text-sm text-white px-2 py-1 rounded border border-gray-600 focus:outline-none focus:border-blue-500"
            />
          </div>
          <div>
            <label className="text-xs text-gray-400 block mb-1">
              Topics (comma-separated, empty = all)
            </label>
            <input
              type="text"
              value={topics}
              onChange={(e) => setTopics(e.target.value)}
              onBlur={() => persistInputs(file, topics)}
              placeholder="/scan, /odom, /tf"
              className="w-full bg-gray-700 text-sm text-white px-2 py-1 rounded border border-gray-600 focus:outline-none focus:border-blue-500"
            />
          </div>
          <div className="flex flex-wrap gap-2 pt-1">
            <button
              onClick={handleStart}
              disabled={sysLoading || !file}
              className="bg-green-600 hover:bg-green-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Start
            </button>
            <button
              onClick={handleStop}
              disabled={sysLoading}
              className="bg-red-600 hover:bg-red-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Stop
            </button>
            <button
              onClick={handleLoadFromEnv}
              disabled={sysLoading}
              className="bg-gray-600 hover:bg-gray-500 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Load from .env
            </button>
          </div>
          {sysError && <p className="text-red-400 text-xs mt-1">{sysError}</p>}
        </div>
      </SectionCard>

      <SectionCard title="Playback Control">
        <div className="space-y-3">
          <div className="flex gap-2">
            <button
              onClick={handlePause}
              disabled={!isRunning || isPaused || rosLoading}
              className="bg-yellow-600 hover:bg-yellow-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Pause
            </button>
            <button
              onClick={handleResume}
              disabled={!isRunning || !isPaused || rosLoading}
              className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
            >
              Resume
            </button>
          </div>
          <div>
            <p className="text-xs text-gray-400 mb-1">Speed</p>
            <div className="flex gap-2">
              {RATE_OPTIONS.map((r) => (
                <button
                  key={r}
                  onClick={() => handleSetRate(r)}
                  disabled={!isRunning || rosLoading}
                  className={`px-3 py-1 rounded text-sm font-medium disabled:opacity-50 ${
                    rate === r
                      ? "bg-blue-600 hover:bg-blue-700"
                      : "bg-gray-600 hover:bg-gray-500"
                  }`}
                >
                  {r}x
                </button>
              ))}
            </div>
          </div>
          {rosError && <p className="text-red-400 text-xs mt-1">{rosError}</p>}
        </div>
      </SectionCard>

      <ValueConfirmDialog
        open={dialogOpen}
        title="Load from .env — 以下の値で上書きしますか？"
        values={[
          { label: "File Path", value: pendingFile },
          { label: "Topics", value: pendingTopics },
        ]}
        onConfirm={handleDialogConfirm}
        onCancel={() => setDialogOpen(false)}
      />
    </div>
  );
}
