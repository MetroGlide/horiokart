import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { Path } from "../../../types/ros-types"

export function usePath(client: FoxgloveClientHandle, topic: string): Path | null {
  const [path, setPath] = useState<Path | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(topic, "nav_msgs/msg/Path", (msg) => {
      setPath(msg as Path)
    })
  }, [client, client.status, topic])

  return path
}
