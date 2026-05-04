import { useMemo, useRef, useEffect } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { usePointCloud2 } from "../hooks/usePointCloud2";
import { TfBuffer } from "../hooks/useTfBuffer";

interface PointCloud2LayerProps {
  client: FoxgloveClientHandle;
  tfBuffer: TfBuffer;
}

// Assumes realsense /camera/camera/depth/color/points format:
// fields: x(f32,0), y(f32,4), z(f32,8), rgb(f32,16), point_step=32
function parseCloud(
  data: number[] | Uint8Array,
  pointStep: number,
  width: number,
  height: number,
  tf: THREE.Matrix4 | null,
): { positions: Float32Array; colors: Float32Array } | null {
  const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = width * height;
  const positions = new Float32Array(count * 3);
  const colors = new Float32Array(count * 3);
  const v = new THREE.Vector3();
  let valid = 0;

  for (let i = 0; i < count; i++) {
    const base = i * pointStep;
    const x = view.getFloat32(base, true);
    const y = view.getFloat32(base + 4, true);
    const z = view.getFloat32(base + 8, true);
    if (!isFinite(x) || !isFinite(y) || !isFinite(z) || z <= 0) continue;

    v.set(x, y, z);
    if (tf) v.applyMatrix4(tf);

    positions[valid * 3] = v.x;
    positions[valid * 3 + 1] = v.y;
    positions[valid * 3 + 2] = v.z;

    // RGB packed in float32 at offset 16
    const rgbInt = view.getUint32(base + 16, true);
    colors[valid * 3] = ((rgbInt >> 16) & 0xff) / 255;
    colors[valid * 3 + 1] = ((rgbInt >> 8) & 0xff) / 255;
    colors[valid * 3 + 2] = (rgbInt & 0xff) / 255;

    valid++;
  }

  return {
    positions: positions.subarray(0, valid * 3),
    colors: colors.subarray(0, valid * 3),
  };
}

export default function PointCloud2Layer({
  client,
  tfBuffer,
}: PointCloud2LayerProps) {
  const cloud = usePointCloud2(client);
  const geomRef = useRef<THREE.BufferGeometry>(null);

  const parsed = useMemo(() => {
    if (!cloud) return null;
    const tf = tfBuffer.lookupTransform("map", cloud.header.frame_id);
    return parseCloud(
      cloud.data,
      cloud.point_step,
      cloud.width,
      cloud.height,
      tf,
    );
  }, [cloud, tfBuffer]); // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!geomRef.current || !parsed) return;
    geomRef.current.setAttribute(
      "position",
      new THREE.BufferAttribute(parsed.positions, 3),
    );
    geomRef.current.setAttribute(
      "color",
      new THREE.BufferAttribute(parsed.colors, 3),
    );
    geomRef.current.setDrawRange(0, parsed.positions.length / 3);
    geomRef.current.computeBoundingSphere();
  }, [parsed]);

  if (!parsed || parsed.positions.length === 0) return null;

  return (
    <points>
      <bufferGeometry ref={geomRef} />
      <pointsMaterial size={0.01} vertexColors sizeAttenuation />
    </points>
  );
}
