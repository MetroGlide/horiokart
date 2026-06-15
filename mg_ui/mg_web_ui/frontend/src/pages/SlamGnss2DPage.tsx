import { useState, useEffect } from "react";
import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { SystemManagerHandle } from "../hooks/useSystemManagerClient";
import { TOPICS } from "../ros/topics";
import RobotPageLayout from "../components/layout/RobotPageLayout";
import SectionCard from "../components/layout/SectionCard";
import SatelliteOverlayViewer, {
  TileType,
  SlamOpacity,
} from "../components/ros-viewer/SatelliteOverlayViewer";
import MapLayer from "../components/ros-viewer/layers/MapLayer";
import PathLine from "../components/ros-viewer/layers/PathLine";
import MarkerArrayLayer from "../components/ros-viewer/layers/MarkerArrayLayer";
import { getSysManagerUrl } from "../utils/systemManagerConfig";

// -------------------------------------------------------------------
// SLAM-GNSS-2D 固有のレイヤー状態
// -------------------------------------------------------------------

interface LayerState {
  poseGraph: boolean;
  gnssRaw: boolean;
  gnssPrior: boolean;
  pathBefore: boolean;
}

// -------------------------------------------------------------------
// レイヤートグルコンポーネント (サイドバー用)
// -------------------------------------------------------------------

interface LayerToggleProps {
  label: string;
  checked: boolean;
  onChange: (v: boolean) => void;
  color: string;
}

function LayerToggle({ label, checked, onChange, color }: LayerToggleProps) {
  return (
    <label className="flex items-center gap-2 cursor-pointer select-none py-0.5">
      <input
        type="checkbox"
        checked={checked}
        onChange={(e) => onChange(e.target.checked)}
        className="w-3.5 h-3.5 rounded"
      />
      <span
        className="text-xs font-mono"
        style={{ color: checked ? color : "#6b7280" }}
      >
        {label}
      </span>
    </label>
  );
}

// -------------------------------------------------------------------
// 透明度ボタングループ
// -------------------------------------------------------------------

const OPACITY_STEPS: SlamOpacity[] = [0.2, 0.4, 0.6, 0.8, 1.0];
const OPACITY_LABELS: Record<SlamOpacity, string> = {
  0.2: "20%",
  0.4: "40%",
  0.6: "60%",
  0.8: "80%",
  1.0: "100%",
};

// -------------------------------------------------------------------
// メインページコンポーネント
// -------------------------------------------------------------------

export default function SlamGnss2DPage({
  client,
  sysManager: _sysManager,
}: {
  client: FoxgloveClientHandle;
  sysManager?: SystemManagerHandle;
}) {
  // SLAM-GNSS-2D 固有レイヤー表示状態
  const [layers, setLayers] = useState<LayerState>({
    poseGraph: true,
    gnssRaw: true,
    gnssPrior: true,
    pathBefore: true,
  });

  // 衛星オーバーレイモード
  const [satelliteMode, setSatelliteMode] = useState(false);
  const [tileType, setTileType] = useState<TileType>("satellite");
  const [slamOpacity, setSlamOpacity] = useState<SlamOpacity>(0.6);

  // Saved SLAM Map / Preview モード
  const [slamMaps, setSlamMaps] = useState<string[]>([]);
  const [selectedMap, setSelectedMap] = useState<string>("");
  const [isPreviewing, setIsPreviewing] = useState(false);
  const [targetDirectory, setTargetDirectory] = useState<string>("/root/ros2_data/slam_maps");
  const [isSaving, setIsSaving] = useState(false);

  const toggleLayer = (key: keyof LayerState) => (v: boolean) =>
    setLayers((prev) => ({ ...prev, [key]: v }));

  const fetchSlamMaps = async (dir?: string) => {
    const fetchDir = dir || targetDirectory;
    try {
      const r = await fetch(
        `${getSysManagerUrl()}/slam_gnss_2d/maps?base_dir=${encodeURIComponent(fetchDir)}`
      );
      const data = await r.json();
      if (data.success && data.maps) {
        setSlamMaps(data.maps);
        if (data.maps.length > 0) {
          if (!data.maps.includes(selectedMap)) {
            setSelectedMap(data.maps[0]);
          }
        } else {
          setSelectedMap("");
        }
      }
    } catch (e) {
      console.error(e);
    }
  };

  useEffect(() => {
    fetchSlamMaps(targetDirectory);
  }, [targetDirectory]);

  const saveSlamMap = async () => {
    if (!_sysManager) return;
    setIsSaving(true);
    try {
      const res = await _sysManager.callApi('/slam_gnss_2d/map/save', {
        output_dir: targetDirectory
      });
      if (res.success) {
        alert("SLAM map saved successfully: " + res.message);
        await fetchSlamMaps(targetDirectory);
      } else {
        console.error("Failed to save SLAM map detailed error:", res.message);
        alert("Failed to save SLAM map: " + res.message);
      }
    } catch (e) {
      console.error("Error saving SLAM map exception:", e);
      alert("Error saving SLAM map: " + String(e));
    } finally {
      setIsSaving(false);
    }
  };

  const startPreview = async () => {
    if (!_sysManager || !selectedMap) return;
    setIsPreviewing(true);
    setSatelliteMode(true); // プレビュー時は自動で衛星モードON
    const fullPath = targetDirectory.endsWith('/') 
      ? `${targetDirectory}${selectedMap}` 
      : `${targetDirectory}/${selectedMap}`;
    await _sysManager.callApi(`/slam_gnss_2d/preview/start`, { slam_map_path: fullPath });
  };

  const stopPreview = async () => {
    if (!_sysManager) return;
    await _sysManager.callApi(`/slam_gnss_2d/preview/stop`, {});
    setIsPreviewing(false);
  };

  // -------------------------------------------------------------------
  // サイドバーアコーディオンアイテム
  // -------------------------------------------------------------------

  const accordionItems = [
    // ── Layers (SLAM-GNSS-2D 固有) ──
    {
      id: "layers",
      label: "Layers",
      children: (
        <SectionCard title="SLAM-GNSS-2D Layers">
          <div className="space-y-1">
            <LayerToggle
              label="Pose Graph"
              checked={layers.poseGraph}
              onChange={toggleLayer("poseGraph")}
              color="#60a5fa"
            />
            <LayerToggle
              label="GNSS Points"
              checked={layers.gnssRaw}
              onChange={toggleLayer("gnssRaw")}
              color="#e879f9"
            />
            <LayerToggle
              label="GNSS Constraints"
              checked={layers.gnssPrior}
              onChange={toggleLayer("gnssPrior")}
              color="#c084fc"
            />
            <LayerToggle
              label="Pre-optimize Path"
              checked={layers.pathBefore}
              onChange={toggleLayer("pathBefore")}
              color="#6b7280"
            />
          </div>
          <p className="text-xs text-gray-500 mt-2">
            ※ 通常 SLAM ビュー時のみ有効
          </p>
        </SectionCard>
      ),
    },

    // ── Saved SLAM Map (新規) ──
    {
      id: "saved-map",
      label: "SLAM Map",
      children: (
        <SectionCard title="SLAM Map Management">
          <div className="space-y-3">
            <div>
              <label className="text-[10px] uppercase tracking-wider text-gray-400 block mb-1">
                Target Directory
              </label>
              <input
                type="text"
                value={targetDirectory}
                onChange={(e) => setTargetDirectory(e.target.value)}
                className="w-full text-xs bg-gray-800 border border-gray-600 rounded p-1.5 focus:outline-none focus:border-blue-500 transition-colors font-mono"
                placeholder="/root/ros2_data/slam_maps"
                disabled={isPreviewing}
              />
            </div>

            <div className="flex gap-2">
              <button
                onClick={saveSlamMap}
                disabled={isSaving || isPreviewing}
                className={`flex-1 text-xs py-1.5 rounded transition-all font-semibold ${
                  isSaving || isPreviewing
                    ? "bg-gray-700 cursor-not-allowed text-gray-500"
                    : "bg-gradient-to-r from-emerald-600 to-teal-500 hover:from-emerald-500 hover:to-teal-400 text-white shadow-md shadow-emerald-950/20"
                }`}
              >
                {isSaving ? "Saving..." : "Save SLAM Map"}
              </button>
              <button
                onClick={() => fetchSlamMaps()}
                disabled={isPreviewing}
                className="text-xs bg-gray-700 hover:bg-gray-600 px-2.5 py-1.5 rounded transition-colors"
              >
                Reload
              </button>
            </div>

            <hr className="border-gray-700 my-2" />

            <div>
              <label className="text-[10px] uppercase tracking-wider text-gray-400 block mb-1">
                Select Map for Preview
              </label>
              {slamMaps.length > 0 ? (
                <select
                  value={selectedMap}
                  onChange={(e) => setSelectedMap(e.target.value)}
                  className="w-full text-xs bg-gray-800 border border-gray-600 rounded p-1.5 focus:outline-none focus:border-blue-500 font-mono"
                  disabled={isPreviewing}
                >
                  {slamMaps.map((m) => (
                    <option key={m} value={m}>
                      {m}
                    </option>
                  ))}
                </select>
              ) : (
                <p className="text-xs text-gray-500 italic">No SLAM maps found in directory</p>
              )}
            </div>

            <div className="flex gap-2">
              <button
                onClick={startPreview}
                disabled={isPreviewing || !selectedMap}
                className={`flex-1 text-xs py-1.5 rounded transition-all font-semibold ${
                  isPreviewing || !selectedMap
                    ? "bg-gray-700 cursor-not-allowed text-gray-500"
                    : "bg-gradient-to-r from-blue-600 to-indigo-600 hover:from-blue-500 hover:to-indigo-500 text-white shadow-md shadow-blue-950/20"
                }`}
              >
                Start Preview
              </button>
              <button
                onClick={stopPreview}
                disabled={!isPreviewing}
                className={`flex-1 text-xs py-1.5 rounded transition-all font-semibold ${
                  !isPreviewing
                    ? "bg-gray-700 cursor-not-allowed text-gray-500"
                    : "bg-gradient-to-r from-red-600 to-pink-600 hover:from-red-500 hover:to-pink-500 text-white shadow-md shadow-red-950/20"
                }`}
              >
                Stop Preview
              </button>
            </div>
          </div>
        </SectionCard>
      ),
    },

    // ── Satellite Overlay (新規) ──
    {
      id: "satellite",
      label: "Satellite Overlay",
      children: (
        <SectionCard title="Satellite Overlay">
          <div className="space-y-3">
            {/* ON/OFF トグル */}
            <div className="flex items-center justify-between">
              <span className="text-xs text-gray-300">衛星オーバーレイ</span>
              <button
                onClick={() => setSatelliteMode((v) => !v)}
                className={`relative inline-flex h-5 w-10 items-center rounded-full transition-colors focus:outline-none ${
                  satelliteMode ? "bg-blue-600" : "bg-gray-600"
                }`}
                role="switch"
                aria-checked={satelliteMode}
              >
                <span
                  className={`inline-block h-4 w-4 transform rounded-full bg-white shadow transition-transform ${
                    satelliteMode ? "translate-x-5" : "translate-x-1"
                  }`}
                />
              </button>
            </div>

            {/* タイル選択 */}
            <div>
              <p className="text-xs text-gray-400 mb-1.5">Map Tile</p>
              <div className="flex gap-1">
                {(["osm", "satellite"] as TileType[]).map((t) => (
                  <button
                    key={t}
                    onClick={() => setTileType(t)}
                    className={`flex-1 text-xs py-1 rounded border transition-colors ${
                      tileType === t
                        ? "bg-blue-600 border-blue-500 text-white"
                        : "bg-gray-700 border-gray-600 text-gray-300 hover:bg-gray-600"
                    }`}
                  >
                    {t === "osm" ? "OSM" : "衛星"}
                  </button>
                ))}
              </div>
            </div>

            {/* SLAM 透明度 */}
            <div>
              <p className="text-xs text-gray-400 mb-1.5">
                SLAM Opacity
              </p>
              <div className="flex gap-1">
                {OPACITY_STEPS.map((op) => (
                  <button
                    key={op}
                    onClick={() => setSlamOpacity(op)}
                    className={`flex-1 text-xs py-1 rounded border transition-colors ${
                      slamOpacity === op
                        ? "bg-indigo-600 border-indigo-500 text-white"
                        : "bg-gray-700 border-gray-600 text-gray-300 hover:bg-gray-600"
                    }`}
                  >
                    {OPACITY_LABELS[op]}
                  </button>
                ))}
              </div>
            </div>

            {satelliteMode && (
              <p className="text-xs text-blue-400 bg-blue-950/40 border border-blue-800/50 rounded px-2 py-1.5">
                衛星地図モード ON<br />
                <span className="text-gray-400">
                  GPS 初回取得位置を SLAM 原点として自動対応
                </span>
              </p>
            )}
          </div>
        </SectionCard>
      ),
    },
  ];

  // -------------------------------------------------------------------
  // 衛星オーバーレイビューワー (モード ON 時に viewerOverride に渡す)
  // -------------------------------------------------------------------

  const viewerOverride = satelliteMode ? (
    <SatelliteOverlayViewer
      client={client}
      tileType={tileType}
      slamOpacity={slamOpacity}
    />
  ) : undefined;

  // -------------------------------------------------------------------
  // 通常 SLAM ビュー用 Three.js シーンの追加レイヤー
  // -------------------------------------------------------------------

  const extraSceneChildren = (
    <>
      {/* SLAM-GNSS-2D マップ (常時表示) */}
      <MapLayer client={client} topic={TOPICS.SLAM_GNSS2D_MAP} />
      {/* 最適化後パス */}
      <PathLine
        client={client}
        topic={TOPICS.SLAM_GNSS2D_PATH}
        color="#00ffff"
        lineWidth={2}
      />
      {/* 最適化前パス */}
      {layers.pathBefore && (
        <PathLine
          client={client}
          topic={TOPICS.SLAM_GNSS2D_PATH_BEFORE}
          color="#666666"
          lineWidth={1}
        />
      )}
      {/* ポーズグラフ */}
      {layers.poseGraph && (
        <MarkerArrayLayer
          client={client}
          topic={TOPICS.SLAM_GNSS2D_POSE_GRAPH}
        />
      )}
      {/* GNSS 生点群 */}
      {layers.gnssRaw && (
        <MarkerArrayLayer
          client={client}
          topic={TOPICS.SLAM_GNSS2D_GNSS_RAW}
        />
      )}
      {/* GNSS 制約 */}
      {layers.gnssPrior && (
        <MarkerArrayLayer
          client={client}
          topic={TOPICS.SLAM_GNSS2D_GNSS_PRIOR}
        />
      )}
    </>
  );

  return (
    <RobotPageLayout
      client={client}
      accordionItems={accordionItems}
      defaultOpen={["layers", "satellite", "saved-map"]}
      viewerMode="2d"
      viewerOverride={viewerOverride}
      extraSceneChildren={satelliteMode ? undefined : extraSceneChildren}
    />
  );
}
