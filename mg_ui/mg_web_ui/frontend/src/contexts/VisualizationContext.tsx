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
  | "cameraImage";

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
  cameraImage: false,
};

interface VisualizationContextType {
  enabled: boolean;
  layers: Record<LayerKey, boolean>;
  setEnabled: (val: boolean) => void;
  toggleLayer: (key: LayerKey) => void;
}

const VisualizationContext = createContext<VisualizationContextType>({
  enabled: true,
  layers: { ...DEFAULT_LAYERS },
  setEnabled: () => {},
  toggleLayer: () => {},
});

export function VisualizationProvider({ children }: { children: ReactNode }) {
  const [enabled, setEnabledState] = useState<boolean>(true);
  const [layers, setLayers] = useState<Record<LayerKey, boolean>>({
    ...DEFAULT_LAYERS,
  });
  const saveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    loadSettings().then((data) => {
      const v = data.visualization as
        | { enabled?: boolean; layers?: Record<string, boolean> }
        | undefined;
      if (!v) return;
      if (typeof v.enabled === "boolean") setEnabledState(v.enabled);
      if (v.layers) setLayers({ ...DEFAULT_LAYERS, ...v.layers });
    });
  }, []);

  const persist = useCallback(
    (nextEnabled: boolean, nextLayers: Record<LayerKey, boolean>) => {
      if (saveTimerRef.current) clearTimeout(saveTimerRef.current);
      saveTimerRef.current = setTimeout(() => {
        saveSettings("visualization", {
          enabled: nextEnabled,
          layers: nextLayers,
        });
      }, 500);
    },
    [],
  );

  const setEnabled = (val: boolean) => {
    setEnabledState(val);
    persist(val, layers);
  };

  const toggleLayer = (key: LayerKey) => {
    setLayers((prev) => {
      const next = { ...prev, [key]: !prev[key] };
      persist(enabled, next);
      return next;
    });
  };

  return (
    <VisualizationContext.Provider
      value={{ enabled, layers, setEnabled, toggleLayer }}
    >
      {children}
    </VisualizationContext.Provider>
  );
}

export function useVisualization() {
  return useContext(VisualizationContext);
}
