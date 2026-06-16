import { useEffect, useRef, useState, useCallback } from "react";
import { Map } from "pigeon-maps";
import proj4 from "proj4";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { OccupancyGrid, NavSatFix } from "../../types/ros";
import { TOPICS } from "../../ros/topics";

// -------------------------------------------------------------------
// タイルプロバイダー定義
// -------------------------------------------------------------------

function osmProvider(x: number, y: number, z: number): string {
  return `https://tile.openstreetmap.org/${z}/${x}/${y}.png`;
}

function satelliteProvider(x: number, y: number, z: number): string {
  // Esri World Imagery (衛星写真)
  return `https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/${z}/${y}/${x}`;
}

// -------------------------------------------------------------------
// メルカトル変換ユーティリティ
// -------------------------------------------------------------------

const TILE_SIZE = 256;

function latLngToWorld(lat: number, lng: number, zoom: number): [number, number] {
  const scale = TILE_SIZE * Math.pow(2, zoom);
  const x = ((lng + 180) / 360) * scale;
  const sinLat = Math.sin((lat * Math.PI) / 180);
  const y = (0.5 - Math.log((1 + sinLat) / (1 - sinLat)) / (4 * Math.PI)) * scale;
  return [x, y];
}

function latLngToPixel(
  lat: number,
  lng: number,
  center: [number, number],
  zoom: number,
  width: number,
  height: number,
): [number, number] {
  const [wx, wy] = latLngToWorld(lat, lng, zoom);
  const [cx, cy] = latLngToWorld(center[0], center[1], zoom);
  return [width / 2 + (wx - cx), height / 2 + (wy - cy)];
}

// -------------------------------------------------------------------
// OccupancyGrid → RGBA ImageData 変換
// -------------------------------------------------------------------

function buildSlamImageData(grid: OccupancyGrid): ImageData {
  const { width, height } = grid.info;
  const data = grid.data;
  const imageData = new ImageData(width, height);
  const buf = imageData.data;

  for (let i = 0; i < width * height; i++) {
    const val = typeof data[i] === "number" ? (data[i] as number) : 0;
    const x = i % width;
    const y = Math.floor(i / width);
    // ROS は下左原点 → 上下反転
    const idx = ((height - 1 - y) * width + x) * 4;

    if (val < 0 || val === 255) {
      // 未知領域: 完全に透明 (枠を見せないため)
      buf[idx] = 128;
      buf[idx + 1] = 128;
      buf[idx + 2] = 128;
      buf[idx + 3] = 0;
    } else if (val === 0) {
      // 自由空間: 白 (完全不透明)
      buf[idx] = 255;
      buf[idx + 1] = 255;
      buf[idx + 2] = 255;
      buf[idx + 3] = 255;
    } else {
      // 障害物: 占有率に応じた暗色 (完全不透明)
      const v = Math.floor((255 * (100 - val)) / 100);
      buf[idx] = v;
      buf[idx + 1] = v;
      buf[idx + 2] = v;
      buf[idx + 3] = 255;
    }
  }

  return imageData;
}

// -------------------------------------------------------------------
// Props
// -------------------------------------------------------------------

export type TileType = "osm" | "satellite";
export type SlamOpacity = 0.2 | 0.4 | 0.6 | 0.8 | 1.0;

interface SatelliteOverlayViewerProps {
  client: FoxgloveClientHandle;
  tileType: TileType;
  slamOpacity: SlamOpacity;
}


// -------------------------------------------------------------------
// メインコンポーネント
// -------------------------------------------------------------------

export default function SatelliteOverlayViewer({
  client,
  tileType,
  slamOpacity,
}: SatelliteOverlayViewerProps) {
  const [mapCenter, setMapCenter] = useState<[number, number]>([35.6895, 139.6917]);
  const [mapZoom, setMapZoom] = useState(17);
  const [mapSize, setMapSize] = useState<{ width: number; height: number }>({
    width: 800,
    height: 600,
  });

  // GPS 位置
  const [gpsFix, setGpsFix] = useState<NavSatFix | null>(null);
  const gpsOriginRef = useRef<{ lat: number; lng: number } | null>(null);

  // SLAM アンカー位置
  const [anchorFix, setAnchorFix] = useState<NavSatFix | null>(null);

  // オフセット微調整パラメータ
  const [offsetX, setOffsetX] = useState<number>(() => {
    const saved = localStorage.getItem("slam_gnss_2d_offset_x");
    return saved ? parseFloat(saved) : 0.0;
  });
  const [offsetY, setOffsetY] = useState<number>(() => {
    const saved = localStorage.getItem("slam_gnss_2d_offset_y");
    return saved ? parseFloat(saved) : 0.0;
  });
  const [offsetYaw, setOffsetYaw] = useState<number>(() => {
    const saved = localStorage.getItem("slam_gnss_2d_offset_yaw");
    return saved ? parseFloat(saved) : 0.0;
  });
  const [offsetScale, setOffsetScale] = useState<number>(() => {
    const saved = localStorage.getItem("slam_gnss_2d_offset_scale");
    return saved ? parseFloat(saved) : 1.0;
  });

  useEffect(() => {
    localStorage.setItem("slam_gnss_2d_offset_x", offsetX.toString());
  }, [offsetX]);

  useEffect(() => {
    localStorage.setItem("slam_gnss_2d_offset_y", offsetY.toString());
  }, [offsetY]);

  useEffect(() => {
    localStorage.setItem("slam_gnss_2d_offset_yaw", offsetYaw.toString());
  }, [offsetYaw]);

  useEffect(() => {
    localStorage.setItem("slam_gnss_2d_offset_scale", offsetScale.toString());
  }, [offsetScale]);

  // SLAM 地図
  const [slamGrid, setSlamGrid] = useState<OccupancyGrid | null>(null);
  const [slamImageData, setSlamImageData] = useState<ImageData | null>(null);

  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  // --- GPS サブスクライブ (client.status を deps に含め、接続時に必ず再実行) ---
  useEffect(() => {
    return client.subscribe(
      TOPICS.GPS_FIX,
      "sensor_msgs/msg/NavSatFix",
      (msg: unknown) => {
        const fix = msg as NavSatFix;
        if (fix.status.status >= 0) {
          setGpsFix(fix);

          // 初回フィックスを GPS 原点として記録
          if (!gpsOriginRef.current && !anchorFix) {
            gpsOriginRef.current = { lat: fix.latitude, lng: fix.longitude };
            setMapCenter([fix.latitude, fix.longitude]);
          }
        }
      },
    );
  }, [client, client.status, anchorFix]);

  // --- SLAM アンカーサブスクライブ ---
  useEffect(() => {
    return client.subscribe(
      "/slam_gnss_2d/anchor",
      "sensor_msgs/msg/NavSatFix",
      (msg: unknown) => {
        const fix = msg as NavSatFix;
        setAnchorFix(fix);
        // アンカーを基準に地図初期位置を設定
        if (!gpsOriginRef.current) {
          gpsOriginRef.current = { lat: fix.latitude, lng: fix.longitude };
          setMapCenter([fix.latitude, fix.longitude]);
        }
      },
    );
  }, [client, client.status]);

  // --- SLAM 地図サブスクライブ ---
  useEffect(() => {
    return client.subscribe(
      TOPICS.SLAM_GNSS2D_MAP,
      "nav_msgs/msg/OccupancyGrid",
      (msg: unknown) => {
        setSlamGrid(msg as OccupancyGrid);
      },
    );
  }, [client, client.status]);

  // --- SLAM 地図 → ImageData 変換 ---
  useEffect(() => {
    if (!slamGrid) return;
    const imageData = buildSlamImageData(slamGrid);
    setSlamImageData(imageData);
  }, [slamGrid]);

  // --- コンテナサイズ監視 ---
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect;
        setMapSize({ width: Math.round(width), height: Math.round(height) });
      }
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // --- Canvas に SLAM 地図を描画 ---
  const drawSlamOverlay = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || !slamGrid || !slamImageData) return;

    // アンカーがなければ正確な重ね合わせができないので描画をスキップ
    const anchor = anchorFix;
    if (!anchor) return;

    const { width: mapW, height: mapH } = mapSize;
    canvas.width = mapW;
    canvas.height = mapH;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.clearRect(0, 0, mapW, mapH);

    const { resolution, width: gridW, height: gridH, origin } = slamGrid.info;

    // UTMパラメータ算出
    const anchorLat = anchor.latitude;
    const anchorLng = anchor.longitude;
    const zone = Math.floor((anchorLng + 180) / 6) + 1;
    const south = anchorLat < 0;

    // proj4定義
    const utmProj = `+proj=utm +zone=${zone}${south ? " +south" : ""} +datum=WGS84 +units=m +no_defs`;
    const wgs84Proj = "+proj=longlat +datum=WGS84 +no_defs";

    // アンカーのUTM座標
    const [anchorUtmX, anchorUtmY] = proj4(wgs84Proj, utmProj, [anchorLng, anchorLat]);

    // 一時 canvas に ImageData を展開
    const tmpCanvas = document.createElement("canvas");
    tmpCanvas.width = gridW;
    tmpCanvas.height = gridH;
    const tmpCtx = tmpCanvas.getContext("2d")!;
    tmpCtx.putImageData(slamImageData, 0, 0);

    const slam_ox = origin.position.x;
    const slam_oy = origin.position.y;

    const rad = (offsetYaw * Math.PI) / 180;
    const cosR = Math.cos(rad);
    const sinR = Math.sin(rad);

    // 4隅の SLAM 座標 (m) → オフセット適用 → UTM 座標 → GPS (WGS84) → ピクセル
    const corners: [number, number][] = (
      [
        { sx: slam_ox, sy: slam_oy }, // 左下 (Bottom-Left)
        { sx: slam_ox + gridW * resolution, sy: slam_oy }, // 右下 (Bottom-Right)
        { sx: slam_ox + gridW * resolution, sy: slam_oy + gridH * resolution }, // 右上 (Top-Right)
        { sx: slam_ox, sy: slam_oy + gridH * resolution }, // 左上 (Top-Left)
      ] as { sx: number; sy: number }[]
    ).map(({ sx, sy }) => {
      // 0. スケール適用
      const scaledX = sx * offsetScale;
      const scaledY = sy * offsetScale;

      // 1. アンカー（原点）周りの回転調整
      const rx = scaledX * cosR - scaledY * sinR;
      const ry = scaledX * sinR + scaledY * cosR;

      // 2. 平行移動オフセット調整
      const tx = rx + offsetX;
      const ty = ry + offsetY;

      // 3. 絶対UTM座標の算出
      const utmX = anchorUtmX + tx;
      const utmY = anchorUtmY + ty;
      const [lng, lat] = proj4(utmProj, wgs84Proj, [utmX, utmY]);
      return latLngToPixel(lat, lng, mapCenter, mapZoom, mapW, mapH);
    });

    const [bl, br, , tl] = corners;

    const dstW = Math.hypot(br[0] - bl[0], br[1] - bl[1]);
    const dstH = Math.hypot(tl[0] - bl[0], tl[1] - bl[1]);

    if (dstW < 1 || dstH < 1) return;

    const angleX = Math.atan2(br[1] - bl[1], br[0] - bl[0]);

    ctx.save();
    ctx.globalAlpha = slamOpacity;
    ctx.translate(bl[0], bl[1]);
    ctx.rotate(angleX);
    ctx.drawImage(tmpCanvas, 0, -dstH, dstW, dstH);
    ctx.restore();
  }, [slamGrid, slamImageData, mapCenter, mapZoom, mapSize, slamOpacity, anchorFix, offsetX, offsetY, offsetYaw, offsetScale]);

  useEffect(() => {
    drawSlamOverlay();
  }, [drawSlamOverlay]);

  const tileProvider = tileType === "satellite" ? satelliteProvider : osmProvider;

  return (
    <div ref={containerRef} className="relative w-full h-full bg-gray-900">
      {mapSize.width > 0 && mapSize.height > 0 && (
        <Map
          center={mapCenter}
          zoom={mapZoom}
          minZoom={3}
          maxZoom={24}
          width={mapSize.width}
          height={mapSize.height}
          provider={tileProvider}
          onBoundsChanged={({ center: c, zoom: z }) => {
            setMapCenter(c);
            setMapZoom(z);
          }}
          attributionPrefix={false}
          attribution={
            tileType === "satellite" ? (
              <span style={{ fontSize: 9, color: "rgba(255,255,255,0.6)" }}>
                Tiles © Esri — Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP
              </span>
            ) : (
              <span style={{ fontSize: 9, color: "rgba(0,0,0,0.5)" }}>
                © OpenStreetMap contributors
              </span>
            )
          }
        />
      )}

      {/* SLAM 地図 Canvas オーバーレイ */}
      <canvas
        ref={canvasRef}
        style={{
          position: "absolute",
          top: 0,
          left: 0,
          pointerEvents: "none",
        }}
      />

      {/* ステータス通知 */}
      {!gpsFix && !anchorFix && (
        <div
          style={{
            position: "absolute",
            top: 8,
            left: "50%",
            transform: "translateX(-50%)",
            background: "rgba(120,80,0,0.85)",
            border: "1px solid rgba(200,150,0,0.7)",
            color: "#fcd34d",
            fontSize: 11,
            fontFamily: "monospace",
            padding: "3px 10px",
            borderRadius: 4,
            pointerEvents: "none",
            backdropFilter: "blur(4px)",
          }}
        >
          GPS / Anchor NO FIX — 衛星マップは初期位置で表示中
        </div>
      )}
      {!anchorFix && gpsFix && (
        <div
          style={{
            position: "absolute",
            top: 8,
            left: "50%",
            transform: "translateX(-50%)",
            background: "rgba(120,80,0,0.85)",
            border: "1px solid rgba(200,150,0,0.7)",
            color: "#fcd34d",
            fontSize: 11,
            fontFamily: "monospace",
            padding: "3px 10px",
            borderRadius: 4,
            pointerEvents: "none",
            backdropFilter: "blur(4px)",
          }}
        >
          Waiting for SLAM GNSS Anchor...
        </div>
      )}
      {!slamGrid && (
        <div
          style={{
            position: "absolute",
            top: (gpsFix || anchorFix) ? 8 : 34,
            left: "50%",
            transform: "translateX(-50%)",
            background: "rgba(30,30,30,0.85)",
            border: "1px solid rgba(100,100,100,0.6)",
            color: "#9ca3af",
            fontSize: 11,
            fontFamily: "monospace",
            padding: "3px 10px",
            borderRadius: 4,
            pointerEvents: "none",
            backdropFilter: "blur(4px)",
          }}
        >
          SLAM Map: waiting... ({TOPICS.SLAM_GNSS2D_MAP})
        </div>
      )}

      {/* オフセット調整パネル */}
      <div
        style={{
          position: "absolute",
          top: 16,
          right: 16,
          background: "rgba(15, 23, 42, 0.85)", // Tailwind Slate 900
          border: "1px solid rgba(100, 116, 139, 0.5)", // Tailwind Slate 500
          borderRadius: 8,
          padding: 14,
          width: 260,
          color: "#f8fafc",
          fontFamily: "sans-serif",
          fontSize: 12,
          zIndex: 10,
          backdropFilter: "blur(8px)",
          boxShadow: "0 4px 6px -1px rgba(0, 0, 0, 0.5), 0 2px 4px -1px rgba(0, 0, 0, 0.5)"
        }}
      >
        <div style={{ fontWeight: 600, fontSize: 13, marginBottom: 12, display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <span>🗺️ 地図オフセット微調整</span>
          <button
            onClick={() => {
              setOffsetX(0);
              setOffsetY(0);
              setOffsetYaw(0);
              setOffsetScale(1.0);
            }}
            style={{
              background: "rgba(100, 116, 139, 0.3)",
              border: "none",
              color: "#cbd5e1",
              fontSize: 10,
              padding: "2px 6px",
              borderRadius: 4,
              cursor: "pointer"
            }}
          >
            Reset
          </button>
        </div>

        <div style={{ display: "flex", flexDirection: "column", gap: 10 }}>
          {/* X offset */}
          <div>
            <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
              <span>X Offset (左右)</span>
              <input
                type="number"
                step="0.1"
                value={offsetX}
                onChange={(e) => setOffsetX(parseFloat(e.target.value) || 0)}
                style={{ width: 70, background: "rgba(0,0,0,0.3)", border: "1px solid rgba(100,116,139,0.5)", color: "#38bdf8", textAlign: "right", borderRadius: 3, fontSize: 11 }}
              />
            </div>
            <input
              type="range"
              min="-500.0"
              max="500.0"
              step="0.1"
              value={offsetX}
              onChange={(e) => setOffsetX(parseFloat(e.target.value))}
              style={{ width: "100%", cursor: "pointer", accentColor: "#0284c7" }}
            />
          </div>

          {/* Y offset */}
          <div>
            <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
              <span>Y Offset (上下)</span>
              <input
                type="number"
                step="0.1"
                value={offsetY}
                onChange={(e) => setOffsetY(parseFloat(e.target.value) || 0)}
                style={{ width: 70, background: "rgba(0,0,0,0.3)", border: "1px solid rgba(100,116,139,0.5)", color: "#38bdf8", textAlign: "right", borderRadius: 3, fontSize: 11 }}
              />
            </div>
            <input
              type="range"
              min="-500.0"
              max="500.0"
              step="0.1"
              value={offsetY}
              onChange={(e) => setOffsetY(parseFloat(e.target.value))}
              style={{ width: "100%", cursor: "pointer", accentColor: "#0284c7" }}
            />
          </div>

          {/* Yaw offset */}
          <div>
            <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
              <span>Rotation (回転)</span>
              <input
                type="number"
                step="0.1"
                value={offsetYaw}
                onChange={(e) => setOffsetYaw(parseFloat(e.target.value) || 0)}
                style={{ width: 70, background: "rgba(0,0,0,0.3)", border: "1px solid rgba(100,116,139,0.5)", color: "#38bdf8", textAlign: "right", borderRadius: 3, fontSize: 11 }}
              />
            </div>
            <input
              type="range"
              min="-180.0"
              max="180.0"
              step="0.1"
              value={offsetYaw}
              onChange={(e) => setOffsetYaw(parseFloat(e.target.value))}
              style={{ width: "100%", cursor: "pointer", accentColor: "#0284c7" }}
            />
          </div>

          {/* Scale offset */}
          <div>
            <div style={{ display: "flex", justifyContent: "space-between", marginBottom: 3 }}>
              <span>Scale (縮尺)</span>
              <input
                type="number"
                step="0.01"
                min="0.01"
                value={offsetScale}
                onChange={(e) => setOffsetScale(parseFloat(e.target.value) || 1.0)}
                style={{ width: 70, background: "rgba(0,0,0,0.3)", border: "1px solid rgba(100,116,139,0.5)", color: "#38bdf8", textAlign: "right", borderRadius: 3, fontSize: 11 }}
              />
            </div>
            <input
              type="range"
              min="0.5"
              max="2.0"
              step="0.01"
              value={offsetScale}
              onChange={(e) => setOffsetScale(parseFloat(e.target.value))}
              style={{ width: "100%", cursor: "pointer", accentColor: "#0284c7" }}
            />
          </div>
        </div>
      </div>

      {/* GPS / Anchor 位置に戻るボタン */}
      {(gpsFix || anchorFix) && (
        <button
          onClick={() => {
            const target = anchorFix || gpsFix;
            if (target) {
              setMapCenter([target.latitude, target.longitude]);
            }
          }}
          style={{
            position: "absolute",
            bottom: 16,
            right: 16,
            background: "rgba(30, 40, 50, 0.85)",
            border: "1px solid rgba(100, 130, 160, 0.7)",
            color: "white",
            fontSize: 11,
            fontWeight: 500,
            padding: "4px 10px",
            borderRadius: 4,
            cursor: "pointer",
            backdropFilter: "blur(4px)",
          }}
          title="現在の基準位置に地図を移動"
        >
          Center Map
        </button>
      )}
    </div>
  );
}
