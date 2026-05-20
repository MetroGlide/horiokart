import { useEffect, useMemo, useRef, useState } from "react";
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

  const draggingRef = useRef(false);
  const dragStartRef = useRef<GroundPoint | null>(null);
  const dragEndRef = useRef<GroundPoint | null>(null);
  const onPoseSetRef = useRef(onPoseSet);
  useEffect(() => {
    onPoseSetRef.current = onPoseSet;
  }, [onPoseSet]);

  useEffect(() => {
    const el = gl.domElement;
    const prevCursor = el.style.cursor;
    el.style.cursor = "crosshair";

    const resolveGroundPoint = (e: MouseEvent): GroundPoint | null => {
      const rect = el.getBoundingClientRect();
      pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const hit = new THREE.Vector3();
      if (!raycaster.ray.intersectPlane(groundPlane, hit)) return null;
      return { x: hit.x, y: hit.y };
    };

    const onDown = (e: MouseEvent) => {
      if (e.button !== 0) return;
      const point = resolveGroundPoint(e);
      if (!point) return;
      e.preventDefault();
      draggingRef.current = true;
      dragStartRef.current = point;
      dragEndRef.current = point;
      setDragStart(point);
      setDragEnd(point);
    };

    const onMove = (e: MouseEvent) => {
      if (!draggingRef.current) return;
      const point = resolveGroundPoint(e);
      if (!point) return;
      dragEndRef.current = point;
      setDragEnd(point);
    };

    const onUp = (e: MouseEvent) => {
      if (!draggingRef.current || !dragStartRef.current) return;
      const point = resolveGroundPoint(e) ?? dragEndRef.current ?? dragStartRef.current;
      const start = dragStartRef.current;
      const dx = point.x - start.x;
      const dy = point.y - start.y;
      const yaw = Math.atan2(dy, dx);
      onPoseSetRef.current(start.x, start.y, yaw);
      draggingRef.current = false;
      dragStartRef.current = null;
      dragEndRef.current = null;
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
  }, [camera, gl, groundPlane, pointer, raycaster]);

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
