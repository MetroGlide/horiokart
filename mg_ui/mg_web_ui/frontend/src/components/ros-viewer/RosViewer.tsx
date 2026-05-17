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
import PoseArrowInteraction, {
  PoseInteractionMode,
} from "./layers/PoseArrowInteraction";

export type ViewerMode = "2d" | "3d";
export type ViewerInteractionMode = "none" | PoseInteractionMode;

function YawControl2D({ enabled = true }: { enabled?: boolean }) {
  const { camera, gl } = useThree();
  const dragging = useRef(false);
  const lastX = useRef(0);
  const touchId = useRef<number | null>(null);
  const lastTouchX = useRef(0);

  useEffect(() => {
    const el = gl.domElement;

    const rotateByDx = (dx: number) => {
      const angle = -dx * 0.005;
      const { x, y } = camera.up;
      const c = Math.cos(angle);
      const s = Math.sin(angle);
      camera.up.set(x * c - y * s, x * s + y * c, 0);
    };

    const onDown = (e: MouseEvent) => {
      if (!enabled) return;
      if (e.button === 0) {
        dragging.current = true;
        lastX.current = e.clientX;
        e.preventDefault();
      }
    };
    const onMove = (e: MouseEvent) => {
      if (!dragging.current) return;
      const dx = e.clientX - lastX.current;
      lastX.current = e.clientX;
      rotateByDx(dx);
    };
    const onUp = () => {
      dragging.current = false;
    };

    const onTouchStart = (e: TouchEvent) => {
      if (!enabled) return;
      if (e.touches.length === 1) {
        touchId.current = e.touches[0].identifier;
        lastTouchX.current = e.touches[0].clientX;
        e.preventDefault();
      }
    };
    const onTouchMove = (e: TouchEvent) => {
      if (touchId.current === null) return;
      if (e.touches.length !== 1) {
        touchId.current = null;
        return;
      }
      const touch = Array.from(e.touches).find(
        (t) => t.identifier === touchId.current
      );
      if (!touch) return;
      const dx = touch.clientX - lastTouchX.current;
      lastTouchX.current = touch.clientX;
      rotateByDx(dx);
      e.preventDefault();
    };
    const onTouchEnd = () => {
      touchId.current = null;
    };

    el.addEventListener("mousedown", onDown);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    el.addEventListener("touchstart", onTouchStart, { passive: false });
    el.addEventListener("touchmove", onTouchMove, { passive: false });
    el.addEventListener("touchend", onTouchEnd);
    el.addEventListener("touchcancel", onTouchEnd);
    return () => {
      el.removeEventListener("mousedown", onDown);
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      el.removeEventListener("touchstart", onTouchStart);
      el.removeEventListener("touchmove", onTouchMove);
      el.removeEventListener("touchend", onTouchEnd);
      el.removeEventListener("touchcancel", onTouchEnd);
    };
  }, [camera, enabled, gl]);

  return null;
}

interface SceneProps {
  client: FoxgloveClientHandle;
  mode: ViewerMode;
  cameraTarget: "map" | "robot";
  interactionMode: ViewerInteractionMode;
  onPoseSet: (x: number, y: number, yaw: number) => void;
  resetToken: number;
}

function CameraFollowRobot({
  tfBuffer,
  mode,
}: {
  tfBuffer: TfBuffer;
  mode: ViewerMode;
}) {
  const { camera, controls: rawControls } = useThree();
  const controls = rawControls as { target: THREE.Vector3 } | null;

  useFrame(() => {
    const mat = tfBuffer.lookupTransform("map", "base_link");
    if (!mat) return;
    const pos = new THREE.Vector3().setFromMatrixPosition(mat);
    if (controls) controls.target.set(pos.x, pos.y, 0);
    if (mode === "2d") {
      camera.position.x = pos.x;
      camera.position.y = pos.y;
    }
  });

  return null;
}

interface CameraState2D {
  position: [number, number, number];
  target: [number, number, number];
  zoom: number;
  up: [number, number, number];
}
interface CameraState3D {
  position: [number, number, number];
  target: [number, number, number];
}

function CameraStatePersistence({ mode }: { mode: ViewerMode }) {
  const { camera, controls: rawControls } = useThree();
  const controls = rawControls as {
    target: THREE.Vector3;
    update: () => void;
    addEventListener: (type: string, cb: () => void) => void;
    removeEventListener: (type: string, cb: () => void) => void;
  } | null;
  const restored = useRef(false);

  useEffect(() => {
    if (!controls || restored.current) return;
    restored.current = true;
    const settingsKey = mode === "2d" ? "viewer_camera_2d" : "viewer_camera_3d";
    loadSettings().then((data) => {
      const saved = data[settingsKey] as
        | CameraState2D
        | CameraState3D
        | undefined;
      if (!saved) return;
      camera.position.fromArray(saved.position);
      controls.target.fromArray(saved.target);
      if (mode === "2d") {
        const s2d = saved as CameraState2D;
        (camera as THREE.OrthographicCamera).zoom = s2d.zoom;
        camera.up.fromArray(s2d.up);
      }
      camera.updateProjectionMatrix();
      controls.update();
    });
  }, [camera, controls, mode]);

  useEffect(() => {
    if (!controls) return;
    const save = () => {
      const settingsKey =
        mode === "2d" ? "viewer_camera_2d" : "viewer_camera_3d";
      const base = {
        position: camera.position.toArray() as [number, number, number],
        target: controls.target.toArray() as [number, number, number],
      };
      const cameraData =
        mode === "2d"
          ? {
              ...base,
              zoom: (camera as THREE.OrthographicCamera).zoom,
              up: camera.up.toArray() as [number, number, number],
            }
          : base;
      saveSettings(settingsKey, cameraData);
    };
    controls.addEventListener("end", save);
    return () => controls.removeEventListener("end", save);
  }, [camera, controls, mode]);

  return null;
}

function CameraResetter({
  mode,
  resetToken,
}: {
  mode: ViewerMode;
  resetToken: number;
}) {
  const { camera, controls: rawControls } = useThree();
  const controls = rawControls as {
    target: THREE.Vector3;
    update: () => void;
  } | null;
  const prevToken = useRef(0);

  useEffect(() => {
    if (resetToken === 0 || resetToken === prevToken.current || !controls)
      return;
    prevToken.current = resetToken;
    if (mode === "2d") {
      camera.position.set(0, 0, 100);
      camera.up.set(0, 1, 0);
      (camera as THREE.OrthographicCamera).zoom = 10;
    } else {
      camera.position.set(0, -20, 20);
    }
    controls.target.set(0, 0, 0);
    camera.updateProjectionMatrix();
    controls.update();
    const settingsKey = mode === "2d" ? "viewer_camera_2d" : "viewer_camera_3d";
    saveSettings(settingsKey, null);
  }, [camera, controls, mode, resetToken]);

  return null;
}

function Scene({
  client,
  mode,
  cameraTarget,
  interactionMode,
  onPoseSet,
  resetToken,
}: SceneProps) {
  const { layers } = useVisualization();
  const tfBuffer = useTfBuffer(client);
  const mapControlsRef = useRef<any>(null);
  const orbitControlsRef = useRef<any>(null);

  useEffect(() => {
    if (mode === "2d" && mapControlsRef.current) {
      mapControlsRef.current.mouseButtons = {
        LEFT: undefined,
        MIDDLE: THREE.MOUSE.PAN,
        RIGHT: undefined,
      };
      mapControlsRef.current.touches = {
        ONE: undefined,
        TWO: THREE.TOUCH.DOLLY_PAN,
      };
    }
  }, [mode, mapControlsRef.current]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (mode === "3d" && orbitControlsRef.current) {
      orbitControlsRef.current.mouseButtons = {
        LEFT: THREE.MOUSE.ROTATE,
        MIDDLE: THREE.MOUSE.PAN,
        RIGHT: undefined,
      };
    }
  }, [mode, orbitControlsRef.current]); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <>
      {mode === "2d" ? (
        <>
          <MapControls
            ref={mapControlsRef}
            makeDefault
            screenSpacePanning
            enabled={interactionMode === "none"}
          />
          <YawControl2D enabled={interactionMode === "none"} />
        </>
      ) : (
        <OrbitControls ref={orbitControlsRef} makeDefault />
      )}
      <CameraStatePersistence mode={mode} />
      <CameraResetter mode={mode} resetToken={resetToken} />
      <ambientLight intensity={1} />
      {cameraTarget === "robot" && (
        <CameraFollowRobot tfBuffer={tfBuffer} mode={mode} />
      )}

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
      {interactionMode !== "none" && (
        <PoseArrowInteraction mode={interactionMode} onPoseSet={onPoseSet} />
      )}
    </>
  );
}

interface RosViewerProps {
  client: FoxgloveClientHandle;
  initialMode?: ViewerMode;
  className?: string;
  interactionMode?: ViewerInteractionMode;
  onPoseSet?: (x: number, y: number, yaw: number) => void;
}

export default function RosViewer({
  client,
  initialMode = "2d",
  className,
  interactionMode = "none",
  onPoseSet,
}: RosViewerProps) {
  const { enabled } = useVisualization();
  const [viewMode, setViewMode] = useState<ViewerMode>(initialMode);
  const [cameraTarget, setCameraTarget] = useState<"map" | "robot">("map");
  const [resetToken, setResetToken] = useState(0);
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

  useEffect(() => {
    if (interactionMode !== "none") {
      setViewMode("2d");
    }
  }, [interactionMode]);

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
          <Scene
            client={client}
            mode={viewMode}
            cameraTarget={cameraTarget}
            interactionMode={interactionMode}
            onPoseSet={onPoseSet ?? (() => {})}
            resetToken={resetToken}
          />
        </Suspense>
      </Canvas>
      <div className="absolute top-2 right-2 flex gap-1 pointer-events-auto">
        <button
          onClick={() => setResetToken((t) => t + 1)}
          className="bg-gray-800/80 hover:bg-gray-700 text-white text-xs font-medium px-2.5 py-1 rounded border border-gray-600 backdrop-blur-sm"
          title="視点をデフォルトに戻す"
        >
          Reset
        </button>
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
