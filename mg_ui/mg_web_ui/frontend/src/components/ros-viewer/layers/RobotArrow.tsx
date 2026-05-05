import { useRef } from "react";
import * as THREE from "three";
import { Line } from "@react-three/drei";
import { useFrame } from "@react-three/fiber";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { TfBuffer } from "../hooks/useTfBuffer";

interface Props {
  client: FoxgloveClientHandle;
  tfBuffer: TfBuffer;
  length?: number;
  width?: number;
}

export default function RobotArrow({
  tfBuffer,
  length = 0.8,
  width = 0.6,
}: Props) {
  const groupRef = useRef<THREE.Group>(null);

  useFrame(() => {
    const tf = tfBuffer.lookupTransform("map", "base_link");
    if (!tf || !groupRef.current) return;
    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    tf.decompose(pos, quat, scale);
    groupRef.current.position.set(pos.x, pos.y, 0.05);
    groupRef.current.rotation.setFromQuaternion(quat);
  });

  const hl = length / 2;
  const hw = width / 2;

  const outline: [number, number, number][] = [
    [hl, hw, 0],
    [hl, -hw, 0],
    [-hl, -hw, 0],
    [-hl, hw, 0],
    [hl, hw, 0],
  ];

  const arrow: [number, number, number][] = [
    [0, 0, 0],
    [hl * 0.8, 0, 0],
  ];

  return (
    <group ref={groupRef}>
      <Line points={outline} color="#00aaff" lineWidth={2} />
      <Line points={arrow} color="#00aaff" lineWidth={3} />
    </group>
  );
}
