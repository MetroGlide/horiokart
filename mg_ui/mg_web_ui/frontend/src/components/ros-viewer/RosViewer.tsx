import { Suspense, useEffect, useRef, useState } from "react";
import * as THREE from "three";
import { Canvas, useFrame } from "@react-three/fiber";
import { MapControls, OrbitControls } from "@react-three/drei";
import { useThree } from "@react-three/fiber";
import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import { useTfBuffer, TfBuffer } from "./hooks/useTfBuffer";
import { TOPICS } from "../../ros/interfaces";
import { loadSettings, saveSettings } from "../../utils/settingsApi";
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

function YawControl2D() {
  const { camera, gl } = useThree();
  const dragging = useRef(false);
  const lastX = useRef(0);

  useEffect(() => {
    const el = gl.domElement;

    const onDown = (e: MouseEvent) => {
      if (e.button === 2) {
        dragging.current = true;
        lastX.current = e.clientX;
        e.preventDefault();
      }
    };
    const onMove = (e: MouseEvent) => {
      if (!dragging.current) return;
      const dx = e.clientX - lastX.current;
      lastX.current = e.clientX;
      const angle = -dx * 0.005;
      const { x, y } = camera.up;
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      camera.up.set(x * c - y * s, x * s + y * c, 0);
    };
    const onUp = () => {
      dragging.current = false;
    };
    const onContext = (e: Event) => e.preventDefault();

    el.addEventListener("contextmenu", onContext);
    el.addEventListener("mousedown", onDown);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    return () => {
      el.removeEventListener("contextmenu", onContext);
      el.removeEventListener("mousedown", onDown);
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
  }, [camera, gl]);

  return null;
}

interface SceneProps {
  client: FoxgloveClientHandle;
  mode: ViewerMode;
  cameraTarget: "map" | "robot";
}

function CameraFollowRobot({ tfBuffer }: { tfBuffer: TfBuffer }) {
  const controls = useThree((state) => state.controls) as {
    target: THREE.Vector3;
  } | null;

  useFrame(() => {
    if (!controls) return;
    const mat = tfBuffer.lookupTransform("map", "base_link");
    if (!mat) return;
    const pos = new THREE.Vector3().setFromMatrixPosition(mat);
    controls.target.set(pos.x, pos.y, 0);
  });

  return null;
}

function Scene({ client, mode, cameraTarget }: SceneProps) {
  const { layers } = useVisualization();
  const tfBuffer = useTfBuffer(client);

  return (
    <>
      {mode === "2d" ? (
        <>
          <MapControls makeDefault screenSpacePanning />
          <YawControl2D />
        </>
      ) : (
        <OrbitControls makeDefault />
      )}
      <ambientLight intensity={1} />
      {cameraTarget === "robot" && <CameraFollowRobot tfBuffer={tfBuffer} />}

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
      {layers.robotPose && <RobotArrow client={client} tfBuffer={tfBuffer} />}
      {layers.particleCloud && <ParticleCloud client={client} />}
      {layers.planPath && (
        <PathLine client={client} topic={TOPICS.NAV_PLAN} color="#ff0000" />
      )}
      {layers.actualPath && (
        <PathLine client={client} topic={TOPICS.ACTUAL_PATH} color="#aa55ff" />
      )}
      {layers.waypointMarkers && <WaypointMarkers client={client} />}
      {layers.collisionPolygons && (
        <CollisionPolygons client={client} tfBuffer={tfBuffer} />
      )}
      {layers.pointCloud && (
        <PointCloud2Layer client={client} tfBuffer={tfBuffer} />
      )}
    </>
  );
}

interface RosViewerProps {
  client: FoxgloveClientHandle;
  initialMode?: ViewerMode;
  className?: string;
}

export default function RosViewer({
  client,
  initialMode = "2d",
  className,
}: RosViewerProps) {
  const { enabled } = useVisualization();
  const [viewMode, setViewMode] = useState<ViewerMode>(initialMode);
  const [cameraTarget, setCameraTarget] = useState<"map" | "robot">("map");
  const settingsLoadedRef = useRef(false);

  useEffect(() => {
    if (settingsLoadedRef.current) return;
    settingsLoadedRef.current = true;
    loadSettings().then((data) => {
      const v = data.viewer as
        | { mode?: string; cameraTarget?: string }
        | undefined;
      if (v?.mode === "2d" || v?.mode === "3d") setViewMode(v.mode);
      if (v?.cameraTarget === "map" || v?.cameraTarget === "robot")
        setCameraTarget(v.cameraTarget);
    });
  }, []);

  const handleSetViewMode = (m: ViewerMode) => {
    setViewMode(m);
    saveSettings("viewer", { mode: m, cameraTarget });
  };

  const handleSetCameraTarget = (t: "map" | "robot") => {
    setCameraTarget(t);
    saveSettings("viewer", { mode: viewMode, cameraTarget: t });
  };

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
    <div className={`relative ${className ?? "w-full h-full"}`}>
      <Canvas
        key={viewMode}
        orthographic={viewMode === "2d"}
        camera={
          viewMode === "2d"
            ? { zoom: 10, position: [0, 0, 100], near: 0.1, far: 10000 }
            : { fov: 60, position: [0, -20, 20], near: 0.1, far: 10000 }
        }
        gl={{ antialias: false }}
      >
        <Suspense fallback={null}>
          <Scene client={client} mode={viewMode} cameraTarget={cameraTarget} />
        </Suspense>
      </Canvas>
      <div className="absolute top-2 right-2 flex gap-1 pointer-events-auto">
        <button
          onClick={() => handleSetViewMode(viewMode === "2d" ? "3d" : "2d")}
          className="bg-gray-800/80 hover:bg-gray-700 text-white text-xs font-medium px-2.5 py-1 rounded border border-gray-600 backdrop-blur-sm"
        >
          {viewMode === "2d" ? "3D" : "2D"}
        </button>
        <button
          onClick={() =>
            handleSetCameraTarget(cameraTarget === "map" ? "robot" : "map")
          }
          className={`text-xs font-medium px-2.5 py-1 rounded border backdrop-blur-sm ${
            cameraTarget === "robot"
              ? "bg-blue-600/80 border-blue-500 text-white"
              : "bg-gray-800/80 border-gray-600 text-white hover:bg-gray-700"
          }`}
        >
          {cameraTarget === "map" ? "Robot Follow" : "Map Fixed"}
        </button>
      </div>
    </div>
  );
}
