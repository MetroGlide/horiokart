import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { OccupancyGrid } from "../../../types/ros-types"

export function useCostmapGrid(
  client: FoxgloveClientHandle,
  topic: string,
): OccupancyGrid | null {
  const [grid, setGrid] = useState<OccupancyGrid | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic, "nav_msgs/msg/OccupancyGrid", (msg) => {
      setGrid(msg as OccupancyGrid)
    })
  }, [client, client.status, topic])

  return grid
}
