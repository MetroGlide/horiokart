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
  setEnabled: (val: boolean) => void;
  toggleLayer: (key: LayerKey) => void;
  toggleOverlay: (key: OverlayKey) => void;
}

const VisualizationContext = createContext<VisualizationContextType>({
  enabled: true,
  layers: { ...DEFAULT_LAYERS },
  overlays: { ...DEFAULT_OVERLAYS },
  setEnabled: () => {},
  toggleLayer: () => {},
  toggleOverlay: () => {},
});

export function VisualizationProvider({ children }: { children: ReactNode }) {
  const [enabled, setEnabledState] = useState<boolean>(true);
  const [layers, setLayers] = useState<Record<LayerKey, boolean>>({
    ...DEFAULT_LAYERS,
  });
  const [overlays, setOverlays] = useState<Record<OverlayKey, boolean>>({
    ...DEFAULT_OVERLAYS,
  });
  const saveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    loadSettings().then((data) => {
      const v = data.visualization as
        | {
            enabled?: boolean;
            layers?: Record<string, boolean>;
            overlays?: Record<string, boolean>;
          }
        | undefined;
      if (!v) return;
      if (typeof v.enabled === "boolean") setEnabledState(v.enabled);
      if (v.layers) setLayers({ ...DEFAULT_LAYERS, ...v.layers });
      if (v.overlays) setOverlays({ ...DEFAULT_OVERLAYS, ...v.overlays });
    });
  }, []);

  const persist = useCallback(
    (
      nextEnabled: boolean,
      nextLayers: Record<LayerKey, boolean>,
      nextOverlays: Record<OverlayKey, boolean>,
    ) => {
      if (saveTimerRef.current) clearTimeout(saveTimerRef.current);
      saveTimerRef.current = setTimeout(() => {
        saveSettings("visualization", {
          enabled: nextEnabled,
          layers: nextLayers,
          overlays: nextOverlays,
        });
      }, 500);
    },
    [],
  );

  const setEnabled = (val: boolean) => {
    setEnabledState(val);
    persist(val, layers, overlays);
  };

  const toggleLayer = (key: LayerKey) => {
    setLayers((prev) => {
      const next = { ...prev, [key]: !prev[key] };
      persist(enabled, next, overlays);
      return next;
    });
  };

  const toggleOverlay = (key: OverlayKey) => {
    setOverlays((prev) => {
      const next = { ...prev, [key]: !prev[key] };
      persist(enabled, layers, next);
      return next;
    });
  };

  return (
    <VisualizationContext.Provider
      value={{
        enabled,
        layers,
        overlays,
        setEnabled,
        toggleLayer,
        toggleOverlay,
      }}
    >
      {children}
    </VisualizationContext.Provider>
  );
}

export function useVisualization() {
  return useContext(VisualizationContext);
}
