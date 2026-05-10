import { useEffect, useState, useMemo } from "react";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { OccupancyGrid } from "../../../types/ros-types";
import { useCostmapGrid } from "../hooks/useCostmapGrid";
import { TfBuffer } from "../hooks/useTfBuffer";

function buildCostmapTexture(grid: OccupancyGrid): THREE.CanvasTexture {
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
    const y = Math.floor(i / width);
    const idx = ((height - 1 - y) * width + x) * 4;

    if (val <= 0) {
      buf[idx + 3] = 0;
    } else if (val === 100) {
      buf[idx] = 255;
      buf[idx + 1] = 0;
      buf[idx + 2] = 0;
      buf[idx + 3] = 255;
    } else {
      const t = val / 100;
      buf[idx] = Math.floor(255 * t);
      buf[idx + 1] = Math.floor(255 * (1 - t));
      buf[idx + 2] = 0;
      buf[idx + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
  const tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.NearestFilter;
  tex.magFilter = THREE.NearestFilter;
  return tex;
}

interface CostmapLayerProps {
  client: FoxgloveClientHandle;
  topic: string;
  opacity?: number;
  tfBuffer?: TfBuffer;
}

export default function CostmapLayer({
  client,
  topic,
  opacity = 0.5,
  tfBuffer,
}: CostmapLayerProps) {
  const grid = useCostmapGrid(client, topic);
  const [texture, setTexture] = useState<THREE.CanvasTexture | null>(null);

  useEffect(() => {
    if (!grid) return;
    const tex = buildCostmapTexture(grid);
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

  const { origin } = grid.info;
  let posX = origin.position.x + geometry.w / 2;
  let posY = origin.position.y + geometry.h / 2;
  let rotZ = 0;

  if (tfBuffer && grid.header.frame_id !== "map") {
    const tf = tfBuffer.lookupTransform("map", grid.header.frame_id);
    if (tf) {
      const v = new THREE.Vector3(posX, posY, 0).applyMatrix4(tf);
      posX = v.x;
      posY = v.y;
      rotZ = new THREE.Euler().setFromRotationMatrix(tf, "XYZ").z;
    }
  }

  return (
    <mesh position={[posX, posY, 0.01]} rotation={[0, 0, rotZ]}>
      <planeGeometry args={[geometry.w, geometry.h]} />
      <meshBasicMaterial
        map={texture}
        transparent
        opacity={opacity}
        depthWrite={false}
      />
    </mesh>
  );
}
