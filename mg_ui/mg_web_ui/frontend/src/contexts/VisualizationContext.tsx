import { createContext, useContext, useState, ReactNode } from "react";

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

function loadFromStorage(): {
  enabled: boolean;
  layers: Record<LayerKey, boolean>;
} {
  try {
    const raw = localStorage.getItem("visualizationSettings");
    if (raw) {
      const parsed = JSON.parse(raw);
      return {
        enabled: typeof parsed.enabled === "boolean" ? parsed.enabled : true,
        layers: { ...DEFAULT_LAYERS, ...(parsed.layers ?? {}) },
      };
    }
  } catch {}
  return { enabled: true, layers: { ...DEFAULT_LAYERS } };
}

export function VisualizationProvider({ children }: { children: ReactNode }) {
  const initial = loadFromStorage();
  const [enabled, setEnabledState] = useState<boolean>(initial.enabled);
  const [layers, setLayers] = useState<Record<LayerKey, boolean>>(
    initial.layers,
  );

  const persist = (
    nextEnabled: boolean,
    nextLayers: Record<LayerKey, boolean>,
  ) => {
    localStorage.setItem(
      "visualizationSettings",
      JSON.stringify({ enabled: nextEnabled, layers: nextLayers }),
    );
  };

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
