import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { PointCloud2 } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function usePointCloud2(client: FoxgloveClientHandle): PointCloud2 | null {
  const [cloud, setCloud] = useState<PointCloud2 | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(
      TOPICS.DEPTH_POINTS,
      "sensor_msgs/msg/PointCloud2",
      (msg) => {
        setCloud(msg as PointCloud2)
      },
    )
  }, [client, client.status])

  return cloud
}
