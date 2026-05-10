import { Text } from "@react-three/drei";
import * as THREE from "three";
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient";
import { Marker, MARKER_TYPE } from "../../../types/ros-types";
import { useMarkerArray } from "../hooks/useMarkerArray";
import { TOPICS } from "../../../ros/interfaces";

function SingleMarker({ marker }: { marker: Marker }) {
  const { pose, scale, color, type, text, points } = marker;
  const { x, y, z } = pose.position;
  const { x: qx, y: qy, z: qz, w: qw } = pose.orientation;
  const rotation = new THREE.Euler().setFromQuaternion(
    new THREE.Quaternion(qx, qy, qz, qw),
  );
  const matColor = new THREE.Color(color.r, color.g, color.b);

  switch (type) {
    case MARKER_TYPE.SPHERE:
      return (
        <mesh position={[x, y, z]} rotation={rotation}>
          <sphereGeometry args={[scale.x / 2, 8, 8]} />
          <meshBasicMaterial color={matColor} transparent opacity={color.a} />
        </mesh>
      );

    case MARKER_TYPE.CUBE:
      return (
        <mesh position={[x, y, z]} rotation={rotation}>
          <boxGeometry args={[scale.x, scale.y, scale.z]} />
          <meshBasicMaterial color={matColor} transparent opacity={color.a} />
        </mesh>
      );

    case MARKER_TYPE.CYLINDER:
      return (
        <mesh position={[x, y, z]} rotation={rotation}>
          <cylinderGeometry args={[scale.x / 2, scale.x / 2, scale.z, 8]} />
          <meshBasicMaterial color={matColor} transparent opacity={color.a} />
        </mesh>
      );

    case MARKER_TYPE.ARROW: {
      const length = scale.x;
      const shaftR = scale.y / 2;
      const headR = scale.z > 0 ? scale.z / 2 : scale.y;
      return (
        <group position={[x, y, z]} rotation={rotation}>
          <group rotation={[0, 0, -Math.PI / 2]}>
            <mesh position={[0, length * 0.35, 0]}>
              <cylinderGeometry args={[shaftR, shaftR, length * 0.7, 8]} />
              <meshBasicMaterial
                color={matColor}
                transparent
                opacity={color.a}
              />
            </mesh>
            <mesh position={[0, length * 0.85, 0]}>
              <coneGeometry args={[headR, length * 0.3, 8]} />
              <meshBasicMaterial
                color={matColor}
                transparent
                opacity={color.a}
              />
            </mesh>
          </group>
        </group>
      );
    }

    case MARKER_TYPE.LINE_STRIP:
    case MARKER_TYPE.LINE_LIST:
      if (points.length < 2) return null;
      return (
        <group position={[x, y, z]}>
          {points.slice(0, -1).map((p, i) => {
            const next = points[i + 1];
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

    case MARKER_TYPE.CUBE_LIST:
      if (points.length === 0) return null;
      return (
        <group>
          {points.map((p, i) => (
            <mesh key={i} position={[p.x, p.y, p.z]} rotation={rotation}>
              <boxGeometry args={[scale.x, scale.y, scale.z]} />
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

    case MARKER_TYPE.POINTS:
      if (points.length === 0) return null;
      return (
        <points>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              args={[
                new Float32Array(points.flatMap((p) => [p.x, p.y, p.z])),
                3,
              ]}
            />
          </bufferGeometry>
          <pointsMaterial
            color={matColor}
            size={scale.x > 0 ? scale.x : 0.1}
            sizeAttenuation
          />
        </points>
      );

    case MARKER_TYPE.TEXT_VIEW_FACING:
      return (
        <Text
          position={[x, y, z + scale.z]}
          fontSize={scale.z}
          color={matColor}
          anchorX="center"
          anchorY="middle"
        >
          {text}
        </Text>
      );

    default:
      return null;
  }
}

export default function WaypointMarkers({
  client,
}: {
  client: FoxgloveClientHandle;
}) {
  const markerArray = useMarkerArray(client, TOPICS.WAYPOINT_MARKERS);

  if (!markerArray || markerArray.markers.length === 0) return null;

  return (
    <>
      {markerArray.markers.map((m) => (
        <SingleMarker key={`${m.ns}_${m.id}`} marker={m} />
      ))}
    </>
  );
}
