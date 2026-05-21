import {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  useRef,
  ReactNode,
} from "react";
import { loadSettings, saveSettings } from "../utils/settingsApi";

export type LayerKey =
  | "map"
  | "globalCostmap"
  | "localCostmap"
  | "lidarTop"
  | "lidarFront"
  | "robotPose"
  | "particleCloud"
  | "planPath"
  | "actualPath"
  | "waypointMarkers"
  | "collisionPolygons"
  | "pointCloud"
  | "colorImage"
  | "depthImage";

export type OverlayKey =
  | "joystick"
  | "velocityGauge"
  | "systemMetrics"
  | "gpsStatus"
  | "gpsMap";

export type GpsMapSize = "default" | "2x-square" | "2x-wide";

const DEFAULT_GPS_MAP_SIZE: GpsMapSize = "default";

const DEFAULT_LAYERS: Record<LayerKey, boolean> = {
  map: true,
  globalCostmap: false,
  localCostmap: false,
  lidarTop: true,
  lidarFront: true,
  robotPose: true,
  particleCloud: false,
  planPath: true,
  actualPath: true,
  waypointMarkers: true,
  collisionPolygons: false,
  pointCloud: false,
  colorImage: false,
  depthImage: false,
};

const DEFAULT_OVERLAYS: Record<OverlayKey, boolean> = {
  joystick: false,
  velocityGauge: true,
  systemMetrics: true,
  gpsStatus: true,
  gpsMap: true,
};

interface VisualizationContextType {
  enabled: boolean;
  layers: Record<LayerKey, boolean>;
  overlays: Record<OverlayKey, boolean>;
  gpsMapSize: GpsMapSize;
  setEnabled: (val: boolean) => void;
  toggleLayer: (key: LayerKey) => void;
  toggleOverlay: (key: OverlayKey) => void;
  setGpsMapSize: (size: GpsMapSize) => void;
}

const VisualizationContext = createContext<VisualizationContextType>({
  enabled: true,
  layers: { ...DEFAULT_LAYERS },
  overlays: { ...DEFAULT_OVERLAYS },
  gpsMapSize: DEFAULT_GPS_MAP_SIZE,
  setEnabled: () => {},
  toggleLayer: () => {},
  toggleOverlay: () => {},
  setGpsMapSize: () => {},
});

export function VisualizationProvider({ children }: { children: ReactNode }) {
  const [enabled, setEnabledState] = useState<boolean>(true);
  const [layers, setLayers] = useState<Record<LayerKey, boolean>>({
    ...DEFAULT_LAYERS,
  });
  const [overlays, setOverlays] = useState<Record<OverlayKey, boolean>>({
    ...DEFAULT_OVERLAYS,
  });
  const [gpsMapSize, setGpsMapSizeState] =
    useState<GpsMapSize>(DEFAULT_GPS_MAP_SIZE);
  const saveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    loadSettings().then((data) => {
      const v = data.visualization as
        | {
            enabled?: boolean;
            layers?: Record<string, boolean>;
            overlays?: Record<string, boolean>;
            gpsMapSize?: string;
          }
        | undefined;
      if (!v) return;
      if (typeof v.enabled === "boolean") setEnabledState(v.enabled);
      if (v.layers) setLayers({ ...DEFAULT_LAYERS, ...v.layers });
      if (v.overlays) setOverlays({ ...DEFAULT_OVERLAYS, ...v.overlays });
      if (
        v.gpsMapSize === "default" ||
        v.gpsMapSize === "2x-square" ||
        v.gpsMapSize === "2x-wide"
      ) {
        setGpsMapSizeState(v.gpsMapSize);
      }
    });
  }, []);

  const persist = useCallback(
    (
      nextEnabled: boolean,
      nextLayers: Record<LayerKey, boolean>,
      nextOverlays: Record<OverlayKey, boolean>,
      nextGpsMapSize: GpsMapSize,
    ) => {
      if (saveTimerRef.current) clearTimeout(saveTimerRef.current);
      saveTimerRef.current = setTimeout(() => {
        saveSettings("visualization", {
          enabled: nextEnabled,
          layers: nextLayers,
          overlays: nextOverlays,
          gpsMapSize: nextGpsMapSize,
        });
      }, 500);
    },
    [],
  );

  const setEnabled = (val: boolean) => {
    setEnabledState(val);
    persist(val, layers, overlays, gpsMapSize);
  };

  const toggleLayer = (key: LayerKey) => {
    setLayers((prev) => {
      const next = { ...prev, [key]: !prev[key] };
      persist(enabled, next, overlays, gpsMapSize);
      return next;
    });
  };

  const toggleOverlay = (key: OverlayKey) => {
    setOverlays((prev) => {
      const next = { ...prev, [key]: !prev[key] };
      persist(enabled, layers, next, gpsMapSize);
      return next;
    });
  };

  const setGpsMapSize = (size: GpsMapSize) => {
    setGpsMapSizeState(size);
    persist(enabled, layers, overlays, size);
  };

  return (
    <VisualizationContext.Provider
      value={{
        enabled,
        layers,
        overlays,
        gpsMapSize,
        setEnabled,
        toggleLayer,
        toggleOverlay,
        setGpsMapSize,
      }}
    >
      {children}
    </VisualizationContext.Provider>
  );
}

export function useVisualization() {
  return useContext(VisualizationContext);
}
