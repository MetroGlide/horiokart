import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { OccupancyGrid } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useOccupancyGrid(client: FoxgloveClientHandle, topic?: string): OccupancyGrid | null {
  const [grid, setGrid] = useState<OccupancyGrid | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic ?? TOPICS.MAP, "nav_msgs/msg/OccupancyGrid", (msg) => {
      setGrid(msg as OccupancyGrid)
    })
  }, [client, client.status, topic])

  return grid
}
