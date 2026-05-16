import { useEffect, useRef, useState } from "react"
import { FoxgloveClientHandle } from "./useFoxgloveClient"
import { useTopicSubscriber } from "./useTopicSubscriber"
import { GoalStatusArray } from "../types"
import { TOPICS, SERVICES } from "../ros/interfaces"

const AMCL_ACTIVE_TIMEOUT_MS = 5000
const LIFECYCLE_POLL_INTERVAL_MS = 5000

export interface Nav2Status {
  navLifecycleActive: boolean
  locLifecycleActive: boolean
  actionStatus: number | null
  amclActive: boolean
}

export function useNav2Status(client: FoxgloveClientHandle): Nav2Status {
  const actionStatusMsg = useTopicSubscriber<GoalStatusArray>(
    client,
    TOPICS.NAV_ACTION_STATUS,
    "action_msgs/msg/GoalStatusArray",
  )

  const amclPoseMsg = useTopicSubscriber<{ header: unknown }>(
    client,
    TOPICS.AMCL_POSE,
    "geometry_msgs/msg/PoseWithCovarianceStamped",
  )

  const [amclActive, setAmclActive] = useState(false)
  const lastAmclRef = useRef<number>(0)

  useEffect(() => {
    if (amclPoseMsg) {
      lastAmclRef.current = Date.now()
      setAmclActive(true)
    }
  }, [amclPoseMsg])

  useEffect(() => {
    const interval = setInterval(() => {
      if (lastAmclRef.current > 0 && Date.now() - lastAmclRef.current > AMCL_ACTIVE_TIMEOUT_MS) {
        setAmclActive(false)
      }
    }, 1000)
    return () => clearInterval(interval)
  }, [])

  const [navLifecycleActive, setNavLifecycleActive] = useState(false)
  const [locLifecycleActive, setLocLifecycleActive] = useState(false)

  useEffect(() => {
    if (client.status !== "connected") {
      setNavLifecycleActive(false)
      setLocLifecycleActive(false)
      return
    }

    const poll = async () => {
      try {
        const r = await client.callService(SERVICES.LIFECYCLE_NAV_IS_ACTIVE, {})
        setNavLifecycleActive((r as { success?: boolean }).success === true)
      } catch {
        setNavLifecycleActive(false)
      }
      try {
        const r = await client.callService(SERVICES.LIFECYCLE_LOC_IS_ACTIVE, {})
        setLocLifecycleActive((r as { success?: boolean }).success === true)
      } catch {
        setLocLifecycleActive(false)
      }
    }

    poll()
    const interval = setInterval(poll, LIFECYCLE_POLL_INTERVAL_MS)
    return () => clearInterval(interval)
  }, [client.status])

  const actionStatus =
    actionStatusMsg && actionStatusMsg.status_list.length > 0
      ? actionStatusMsg.status_list[actionStatusMsg.status_list.length - 1].status
      : null

  return { navLifecycleActive, locLifecycleActive, actionStatus, amclActive }
}
