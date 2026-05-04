import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { LaserScan } from "../../../types/ros-types"

export function useLaserScan(
  client: FoxgloveClientHandle,
  topic: string,
): LaserScan | null {
  const [scan, setScan] = useState<LaserScan | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic, "sensor_msgs/msg/LaserScan", (msg) => {
      setScan(msg as LaserScan)
    })
  }, [client, client.status, topic])

  return scan
}
