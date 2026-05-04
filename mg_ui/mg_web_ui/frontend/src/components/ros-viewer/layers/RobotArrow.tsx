import { useMemo } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useRobotPose } from "../hooks/useRobotPose";

export default function RobotArrow({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const pose = useRobotPose(client);

  const rotation = useMemo(() => {
    if (!pose) return new THREE.Euler();
    const { x, y, z, w } = pose.pose.pose.orientation;
    return new THREE.Euler().setFromQuaternion(
      new THREE.Quaternion(x, y, z, w),
    );
  }, [pose]);

  if (!pose) return null;

  const { x, y } = pose.pose.pose.position;

  return (
    <group position={[x, y, 0.05]} rotation={rotation}>
      {/* shaft */}
      <mesh position={[0, 0.15, 0]}>
        <cylinderGeometry args={[0.08, 0.08, 0.3, 8]} />
        <meshBasicMaterial color="#00aaff" />
      </mesh>
      {/* arrowhead */}
      <mesh position={[0, 0.4, 0]}>
        <coneGeometry args={[0.15, 0.3, 8]} />
        <meshBasicMaterial color="#00aaff" />
      </mesh>
    </group>
  );
}
