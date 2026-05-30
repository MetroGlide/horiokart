import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { Marker, MARKER_TYPE } from "../../../types/ros-types";
import { useMarkerArray } from "../hooks/useMarkerArray";

function SingleMarker({ marker }: { marker: Marker }) {
  const { scale, color, type, points } = marker;
  const matColor = new THREE.Color(color.r, color.g, color.b);

  switch (type) {
    case MARKER_TYPE.SPHERE_LIST:
      if (points.length === 0) return null;
      return (
        <group>
          {points.map((p, i) => (
            <mesh key={i} position={[p.x, p.y, p.z]}>
              <sphereGeometry args={[scale.x / 2, 8, 8]} />
              <meshBasicMaterial
                color={
                  marker.colors[i]
                    ? new THREE.Color(
                        marker.colors[i].r,
                        marker.colors[i].g,
                        marker.colors[i].b,
                      )
                    : matColor
                }
                transparent
                opacity={marker.colors[i] ? marker.colors[i].a : color.a}
              />
            </mesh>
          ))}
        </group>
      );

    case MARKER_TYPE.LINE_LIST:
      if (points.length < 2) return null;
      return (
        <group>
          {Array.from({ length: Math.floor(points.length / 2) }, (_, i) => {
            const p = points[i * 2];
            const next = points[i * 2 + 1];
            return (
              <line key={i}>
                <bufferGeometry>
                  <bufferAttribute
                    attach="attributes-position"
                    args={[
                      new Float32Array([p.x, p.y, p.z, next.x, next.y, next.z]),
                      3,
                    ]}
                  />
                </bufferGeometry>
                <lineBasicMaterial color={matColor} />
              </line>
            );
          })}
        </group>
      );

    default:
      return null;
  }
}

interface MarkerArrayLayerProps {
  client: FoxgloveClientHandle;
  topic: string;
}

export default function MarkerArrayLayer({
  client,
  topic,
}: MarkerArrayLayerProps) {
  const markerArray = useMarkerArray(client, topic);

  if (!markerArray || markerArray.markers.length === 0) return null;

  return (
    <>
      {markerArray.markers.map((m) => (
        <SingleMarker key={`${m.ns}_${m.id}`} marker={m} />
      ))}
    </>
  );
}
