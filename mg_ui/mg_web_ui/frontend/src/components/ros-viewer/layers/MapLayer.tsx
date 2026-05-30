import { useMemo, useEffect, useState } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { OccupancyGrid } from "../../../types/ros-types";
import { useOccupancyGrid } from "../hooks/useOccupancyGrid";

function buildMapTexture(grid: OccupancyGrid): THREE.CanvasTexture {
  const { width, height } = grid.info;
  const data = grid.data;
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext("2d")!;
  const imageData = ctx.createImageData(width, height);
  const buf = imageData.data;

  for (let i = 0; i < width * height; i++) {
    const val = data[i];
    const x = i % width;
    // ROS map origin is bottom-left; canvas origin is top-left → flip y
    const y = Math.floor(i / width);
    const idx = ((height - 1 - y) * width + x) * 4;

    if (val < 0 || val === 255) {
      buf[idx] = 127;
      buf[idx + 1] = 127;
      buf[idx + 2] = 127;
      buf[idx + 3] = 255;
    } else if (val === 0) {
      buf[idx] = 210;
      buf[idx + 1] = 210;
      buf[idx + 2] = 210;
      buf[idx + 3] = 255;
    } else {
      const v = Math.floor((255 * (100 - val)) / 100);
      buf[idx] = v;
      buf[idx + 1] = v;
      buf[idx + 2] = v;
      buf[idx + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
  const tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.NearestFilter;
  tex.magFilter = THREE.NearestFilter;
  return tex;
}

export default function MapLayer({
  client,
  topic,
}: {
  client: FoxgloveClientHandle;
  topic?: string;
}) {
  const grid = useOccupancyGrid(client, topic);
  const [texture, setTexture] = useState<THREE.CanvasTexture | null>(null);

  useEffect(() => {
    if (!grid) return;
    const tex = buildMapTexture(grid);
    setTexture((prev) => {
      prev?.dispose();
      return tex;
    });
    return () => tex.dispose();
  }, [grid]);

  const geometry = useMemo(() => {
    if (!grid) return null;
    const { resolution, width, height } = grid.info;
    return { w: width * resolution, h: height * resolution };
  }, [grid]);

  if (!grid || !texture || !geometry) return null;

  const { resolution, origin } = grid.info;
  const cx = origin.position.x + geometry.w / 2;
  const cy = origin.position.y + geometry.h / 2;

  return (
    <mesh position={[cx, cy, -0.01]}>
      <planeGeometry args={[geometry.w, geometry.h]} />
      <meshBasicMaterial map={texture} depthWrite={false} />
    </mesh>
  );
}
