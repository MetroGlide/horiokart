import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { PolygonStamped } from "../../../types/ros-types"

export function usePolygonStamped(
  client: FoxgloveClientHandle,
  topic: string,
): PolygonStamped | null {
  const [polygon, setPolygon] = useState<PolygonStamped | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic, "geometry_msgs/msg/PolygonStamped", (msg) => {
      setPolygon(msg as PolygonStamped)
    })
  }, [client, client.status, topic])

  return polygon
}
