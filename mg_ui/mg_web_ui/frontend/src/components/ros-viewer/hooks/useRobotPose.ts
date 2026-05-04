import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { PoseWithCovarianceStamped } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useRobotPose(client: FoxgloveClientHandle): PoseWithCovarianceStamped | null {
  const [pose, setPose] = useState<PoseWithCovarianceStamped | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return
    return client.subscribe(
      TOPICS.AMCL_POSE,
      "geometry_msgs/msg/PoseWithCovarianceStamped",
      (msg) => {
        setPose(msg as PoseWithCovarianceStamped)
      },
    )
  }, [client, client.status])

  return pose
}
