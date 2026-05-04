import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { MapControls, OrbitControls } from "@react-three/drei";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import { useTfBuffer } from "./hooks/useTfBuffer";
import { TOPICS } from "../../ros/interfaces";
import MapLayer from "./layers/MapLayer";
import CostmapLayer from "./layers/CostmapLayer";
import LaserScanLayer from "./layers/LaserScanLayer";
import RobotArrow from "./layers/RobotArrow";
import PathLine from "./layers/PathLine";
import WaypointMarkers from "./layers/WaypointMarkers";
import CollisionPolygons from "./layers/CollisionPolygons";
import ParticleCloud from "./layers/ParticleCloud";
import PointCloud2Layer from "./layers/PointCloud2Layer";

export type ViewerMode = "2d" | "3d";

interface SceneProps {
  client: FoxgloveClientHandle;
  mode: ViewerMode;
}

function Scene({ client, mode }: SceneProps) {
  const { layers } = useVisualization();
  const tfBuffer = useTfBuffer(client);

  return (
    <>
      {mode === "2d" ? (
        <MapControls makeDefault screenSpacePanning />
      ) : (
        <OrbitControls makeDefault />
      )}
      <ambientLight intensity={1} />

      {layers.map && <MapLayer client={client} />}
      {layers.globalCostmap && (
        <CostmapLayer
          client={client}
          topic={TOPICS.GLOBAL_COSTMAP}
          opacity={0.3}
          tfBuffer={tfBuffer}
        />
      )}
      {layers.localCostmap && (
        <CostmapLayer
          client={client}
          topic={TOPICS.LOCAL_COSTMAP}
          opacity={0.5}
          tfBuffer={tfBuffer}
        />
      )}
      {layers.lidarTop && (
        <LaserScanLayer
          client={client}
          topic={TOPICS.SCAN_TOP}
          color="#00ffff"
          tfBuffer={tfBuffer}
        />
      )}
      {layers.lidarFront && (
        <LaserScanLayer
          client={client}
          topic={TOPICS.SCAN_FRONT}
          color="#00ff80"
          tfBuffer={tfBuffer}
        />
      )}
      {layers.robotPose && <RobotArrow client={client} />}
      {layers.particleCloud && <ParticleCloud client={client} />}
      {layers.planPath && (
        <PathLine client={client} topic={TOPICS.NAV_PLAN} color="#ff0000" />
      )}
      {layers.actualPath && (
        <PathLine client={client} topic={TOPICS.ACTUAL_PATH} color="#aa55ff" />
      )}
      {layers.waypointMarkers && <WaypointMarkers client={client} />}
      {layers.collisionPolygons && <CollisionPolygons client={client} />}
      {layers.pointCloud && (
        <PointCloud2Layer client={client} tfBuffer={tfBuffer} />
      )}
    </>
  );
}

interface RosViewerProps {
  client: FoxgloveClientHandle;
  mode: ViewerMode;
  className?: string;
}

export default function RosViewer({ client, mode, className }: RosViewerProps) {
  const { enabled } = useVisualization();

  if (!enabled) {
    return (
      <div
        className={`flex items-center justify-center bg-gray-900 ${className ?? "w-full h-full"}`}
      >
        <span className="text-gray-500 text-sm">
          Visualization is disabled. Enable it in Settings.
        </span>
      </div>
    );
  }

  return (
    <div className={className ?? "w-full h-full"}>
      <Canvas
        orthographic={mode === "2d"}
        camera={
          mode === "2d"
            ? { zoom: 10, position: [0, 0, 100], near: 0.1, far: 10000 }
            : { fov: 60, position: [0, -20, 20], near: 0.1, far: 10000 }
        }
        gl={{ antialias: false }}
      >
        <Suspense fallback={null}>
          <Scene client={client} mode={mode} />
        </Suspense>
      </Canvas>
    </div>
  );
}
