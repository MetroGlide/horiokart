import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { ParticleCloud } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useParticleCloud(client: FoxgloveClientHandle): ParticleCloud | null {
  const [cloud, setCloud] = useState<ParticleCloud | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(
      TOPICS.PARTICLE_CLOUD,
      "nav2_msgs/msg/ParticleCloud",
      (msg) => {
        setCloud(msg as ParticleCloud)
      },
    )
  }, [client, client.status])

  return cloud
}
