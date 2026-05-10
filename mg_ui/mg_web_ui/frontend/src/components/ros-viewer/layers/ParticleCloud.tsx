import { useMemo } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useParticleCloud } from "../hooks/useParticleCloud";

export default function ParticleCloud({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const cloud = useParticleCloud(client);

  const positions = useMemo(() => {
    if (!cloud || !cloud.particles?.length) return null;
    const arr = new Float32Array(cloud.particles.length * 3);
    cloud.particles.forEach(({ pose }, i) => {
      arr[i * 3] = pose.position.x;
      arr[i * 3 + 1] = pose.position.y;
      arr[i * 3 + 2] = pose.position.z;
    });
    return arr;
  }, [cloud]);

  if (!positions) return null;

  return (
    <points>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" args={[positions, 3]} />
      </bufferGeometry>
      <pointsMaterial color="#00b400" size={0.1} sizeAttenuation />
    </points>
  );
}
