import { useMemo, useRef, useEffect } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { useLaserScan } from "../hooks/useLaserScan";
import { TfBuffer } from "../hooks/useTfBuffer";

interface LaserScanLayerProps {
  client: FoxgloveClientHandle;
  topic: string;
  color?: string;
  tfBuffer: TfBuffer;
}

export default function LaserScanLayer({
  client,
  topic,
  color = "#ffffff",
  tfBuffer,
}: LaserScanLayerProps) {
  const scan = useLaserScan(client, topic);
  const geomRef = useRef<THREE.BufferGeometry>(null);

  // Transform scan points to map frame whenever a new scan arrives.
  // tfBuffer.lookupTransform reads from a ref, so always returns the latest TF.
  const positions = useMemo(() => {
    if (!scan) return null;

    const { ranges, angle_min, angle_increment, range_min, range_max, header } =
      scan;
    const tf = tfBuffer.lookupTransform("map", header.frame_id);

    const local: number[] = [];
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (!isFinite(r) || r < range_min || r > range_max) continue;
      const angle = angle_min + i * angle_increment;
      local.push(r * Math.cos(angle), r * Math.sin(angle), 0);
    }

    if (!tf) return new Float32Array(local);

    const v = new THREE.Vector3();
    const out: number[] = [];
    for (let i = 0; i < local.length; i += 3) {
      v.set(local[i], local[i + 1], local[i + 2]).applyMatrix4(tf);
      out.push(v.x, v.y, v.z);
    }
    return new Float32Array(out);
  }, [scan, tfBuffer]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!geomRef.current || !positions) return;
    geomRef.current.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3),
    );
    geomRef.current.setDrawRange(0, positions.length / 3);
    geomRef.current.computeBoundingSphere();
  }, [positions]);

  if (!positions || positions.length === 0) return null;

  return (
    <points>
      <bufferGeometry ref={geomRef} />
      <pointsMaterial color={color} size={0.05} sizeAttenuation />
    </points>
  );
}
