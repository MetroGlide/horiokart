import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { MarkerArray } from "../../../types/ros-types"

export function useMarkerArray(
  client: FoxgloveClientHandle,
  topic: string,
): MarkerArray | null {
  const [markers, setMarkers] = useState<MarkerArray | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic, "visualization_msgs/msg/MarkerArray", (msg) => {
      setMarkers(msg as MarkerArray)
    })
  }, [client, client.status, topic])

  return markers
}
