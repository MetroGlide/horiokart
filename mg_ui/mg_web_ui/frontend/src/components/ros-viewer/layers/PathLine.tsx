import { useMemo } from "react";
import { Line } from "@react-three/drei";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { usePath } from "../hooks/usePath";

interface PathLineProps {
  client: FoxgloveClientHandle;
  topic: string;
  color: string;
  lineWidth?: number;
}

export default function PathLine({
  client,
  topic,
  color,
  lineWidth = 2,
}: PathLineProps) {
  const path = usePath(client, topic);

  const points = useMemo(() => {
    if (!path || path.poses.length < 2) return null;
    return path.poses.map((p) => {
      const { x, y, z } = p.pose.position;
      return [x, y, z] as [number, number, number];
    });
  }, [path]);

  if (!points) return null;

  return <Line points={points} color={color} lineWidth={lineWidth} />;
}
