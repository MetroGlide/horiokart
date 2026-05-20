import { useEffect, useState } from "react";
import { Map, Marker } from "pigeon-maps";
import { NavSatFix } from "../../types/ros";

const MAP_SIZE = 192;
const DEFAULT_CENTER: [number, number] = [35.6895, 139.6917];
const DEFAULT_ZOOM = 17;

function mercatorProject(
  lat: number,
  lng: number,
  center: [number, number],
  zoom: number,
): [number, number] {
  const TILE = 256;
  const scale = TILE * Math.pow(2, zoom);
  const toMx = (l: number) => ((l + 180) / 360) * scale;
  const toMy = (l: number) => {
    const s = Math.sin((l * Math.PI) / 180);
    return (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * scale;
  };
  return [
    MAP_SIZE / 2 + (toMx(lng) - toMx(center[1])),
    MAP_SIZE / 2 + (toMy(lat) - toMy(center[0])),
  ];
}

interface GpsMapOverlayProps {
  fix: NavSatFix | null;
  trail: [number, number][];
}

export default function GpsMapOverlay({ fix, trail }: GpsMapOverlayProps) {
  const hasFix = fix !== null && fix.status.status >= 0;
  const position: [number, number] | undefined = hasFix
    ? [fix.latitude, fix.longitude]
    : undefined;

  const [center, setCenter] = useState<[number, number]>(DEFAULT_CENTER);
  const [zoom, setZoom] = useState(DEFAULT_ZOOM);

  useEffect(() => {
    if (position) {
      setCenter(position);
    }
  }, [fix?.latitude, fix?.longitude]);

  const pathD =
    trail.length >= 2
      ? trail
          .map(([lat, lng], i) => {
            const [x, y] = mercatorProject(lat, lng, center, zoom);
            return `${i === 0 ? "M" : "L"}${x.toFixed(1)},${y.toFixed(1)}`;
          })
          .join(" ")
      : null;

  return (
    <div
      className="w-48 h-48 rounded-lg overflow-hidden border border-gray-600/50 shadow-lg"
      style={{ position: "relative" }}
    >
      <Map
        center={center}
        zoom={zoom}
        width={MAP_SIZE}
        height={MAP_SIZE}
        onBoundsChanged={({ center: c, zoom: z }) => {
          setCenter(c);
          setZoom(z);
        }}
        attributionPrefix={false}
      >
        {position && <Marker anchor={position} width={20} color="#00ccff" />}
      </Map>
      {pathD && (
        <svg
          style={{
            position: "absolute",
            top: 0,
            left: 0,
            width: MAP_SIZE,
            height: MAP_SIZE,
            pointerEvents: "none",
          }}
        >
          <path
            d={pathD}
            fill="none"
            stroke="#00ff88"
            strokeWidth={2}
            strokeLinejoin="round"
            strokeLinecap="round"
            opacity={0.8}
          />
        </svg>
      )}
      {!hasFix && (
        <div className="absolute inset-0 bg-gray-900/60 flex items-center justify-center pointer-events-none">
          <span className="text-xs text-gray-400 font-mono">NO FIX</span>
        </div>
      )}
    </div>
  );
}
