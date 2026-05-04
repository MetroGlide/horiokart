import { Line } from "@react-three/drei";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { usePolygonStamped } from "../hooks/usePolygonStamped";
import { TOPICS } from "../../../ros/interfaces";

function PolygonOutline({
  client,
  topic,
  color,
}: {
  client: FoxgloveClientHandle;
  topic: string;
  color: string;
}) {
  const poly = usePolygonStamped(client, topic);

  if (!poly || poly.polygon.points.length < 2) return null;

  const pts = poly.polygon.points;
  const closed: [number, number, number][] = [
    ...pts.map((p) => [p.x, p.y, p.z] as [number, number, number]),
    [pts[0].x, pts[0].y, pts[0].z],
  ];

  return <Line points={closed} color={color} lineWidth={2} />;
}

export default function CollisionPolygons({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  return (
    <>
      <PolygonOutline
        client={client}
        topic={TOPICS.COLLISION_FRONT}
        color="#aa0088"
      />
      <PolygonOutline
        client={client}
        topic={TOPICS.COLLISION_REAR}
        color="#aa0088"
      />
    </>
  );
}
