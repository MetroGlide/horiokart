import { Line } from "@react-three/drei";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { usePolygonStamped } from "../hooks/usePolygonStamped";
import { TfBuffer } from "../hooks/useTfBuffer";
import { TOPICS } from "../../../ros/interfaces";

function PolygonOutline({
  client,
  topic,
  color,
  tfBuffer,
}: {
  client: FoxgloveClientHandle;
  topic: string;
  color: string;
  tfBuffer?: TfBuffer;
}) {
  const poly = usePolygonStamped(client, topic);

  if (!poly || poly.polygon.points.length < 2) return null;

  let pts = poly.polygon.points.map(
    (p) => [p.x, p.y, p.z] as [number, number, number],
  );

  if (tfBuffer && poly.header.frame_id && poly.header.frame_id !== "map") {
    const tf = tfBuffer.lookupTransform("map", poly.header.frame_id);
    if (tf) {
      pts = pts.map(([px, py, pz]) => {
        const v = new THREE.Vector3(px, py, pz).applyMatrix4(tf);
        return [v.x, v.y, v.z] as [number, number, number];
      });
    }
  }

  const closed: [number, number, number][] = [...pts, pts[0]];
  return <Line points={closed} color={color} lineWidth={2} />;
}

export default function CollisionPolygons({
  client,
  tfBuffer,
}: {
  client: FoxgloveClientHandle;
  tfBuffer?: TfBuffer;
}) {
  return (
    <>
      <PolygonOutline
        client={client}
        topic={TOPICS.COLLISION_FRONT}
        color="#aa0088"
        tfBuffer={tfBuffer}
      />
      <PolygonOutline
        client={client}
        topic={TOPICS.COLLISION_REAR}
        color="#aa0088"
        tfBuffer={tfBuffer}
      />
    </>
  );
}
