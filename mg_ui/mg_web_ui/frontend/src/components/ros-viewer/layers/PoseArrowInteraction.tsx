import { useEffect, useMemo, useState } from "react";
import * as THREE from "three";
import { useThree } from "@react-three/fiber";

export type PoseInteractionMode = "pose_estimate" | "nav_goal";

interface PoseArrowInteractionProps {
  mode: PoseInteractionMode;
  onPoseSet: (x: number, y: number, yaw: number) => void;
}

interface GroundPoint {
  x: number;
  y: number;
}

export default function PoseArrowInteraction({
  mode,
  onPoseSet,
}: PoseArrowInteractionProps) {
  const { camera, gl } = useThree();
  const raycaster = useMemo(() => new THREE.Raycaster(), []);
  const pointer = useMemo(() => new THREE.Vector2(), []);
  const groundPlane = useMemo(
    () => new THREE.Plane(new THREE.Vector3(0, 0, 1), 0),
    [],
  );
  const [dragStart, setDragStart] = useState<GroundPoint | null>(null);
  const [dragEnd, setDragEnd] = useState<GroundPoint | null>(null);
  const [dragging, setDragging] = useState(false);

  const resolveGroundPoint = (e: MouseEvent): GroundPoint | null => {
    const rect = gl.domElement.getBoundingClientRect();
    pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(groundPlane, hit)) return null;
    return { x: hit.x, y: hit.y };
  };

  useEffect(() => {
    const el = gl.domElement;
    const prevCursor = el.style.cursor;
    el.style.cursor = "crosshair";

    const onDown = (e: MouseEvent) => {
      if (e.button !== 0) return;
      const point = resolveGroundPoint(e);
      if (!point) return;
      e.preventDefault();
      setDragging(true);
      setDragStart(point);
      setDragEnd(point);
    };

    const onMove = (e: MouseEvent) => {
      if (!dragging) return;
      const point = resolveGroundPoint(e);
      if (!point) return;
      setDragEnd(point);
    };

    const onUp = (e: MouseEvent) => {
      if (!dragging || !dragStart) return;
      const point = resolveGroundPoint(e) ?? dragEnd ?? dragStart;
      const dx = point.x - dragStart.x;
      const dy = point.y - dragStart.y;
      const yaw = Math.atan2(dy, dx);
      onPoseSet(dragStart.x, dragStart.y, yaw);
      setDragging(false);
      setDragStart(null);
      setDragEnd(null);
    };

    el.addEventListener("mousedown", onDown);
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);

    return () => {
      el.style.cursor = prevCursor;
      el.removeEventListener("mousedown", onDown);
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
    };
  }, [
    camera,
    dragEnd,
    dragStart,
    dragging,
    gl,
    groundPlane,
    onPoseSet,
    pointer,
    raycaster,
  ]);

  if (!dragStart || !dragEnd) return null;

  const dx = dragEnd.x - dragStart.x;
  const dy = dragEnd.y - dragStart.y;
  const length = Math.max(Math.hypot(dx, dy), 0.001);
  const yaw = Math.atan2(dy, dx);
  const color = mode === "pose_estimate" ? "#facc15" : "#f97316";
  const headLength = Math.min(0.4, Math.max(0.18, length * 0.3));
  const stemLength = Math.max(0.06, length - headLength);

  return (
    <group position={[dragStart.x, dragStart.y, 0.05]} rotation={[0, 0, yaw]}>
      <mesh position={[stemLength / 2, 0, 0]}>
        <boxGeometry args={[stemLength, 0.08, 0.03]} />
        <meshBasicMaterial color={color} transparent opacity={0.9} />
      </mesh>
      <mesh
        position={[stemLength + headLength / 2, 0, 0]}
        rotation={[0, 0, -Math.PI / 2]}
      >
        <coneGeometry args={[0.12, headLength, 16]} />
        <meshBasicMaterial color={color} transparent opacity={0.95} />
      </mesh>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[0.07, 16, 16]} />
        <meshBasicMaterial color={color} transparent opacity={0.95} />
      </mesh>
    </group>
  );
}
