import { useEffect, useState } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'
import { useTopicSubscriber } from './useTopicSubscriber'
import { NavSatFix } from '../types/ros'
import { TOPICS } from '../ros/topics'

const MAX_TRAIL_POINTS = 200

export function useGpsFix(client: FoxgloveClientHandle): {
  fix: NavSatFix | null
  trail: [number, number][]
} {
  const fix = useTopicSubscriber<NavSatFix>(
    client,
    TOPICS.GPS_FIX,
    'sensor_msgs/msg/NavSatFix',
  )
  const [trail, setTrail] = useState<[number, number][]>([])

  useEffect(() => {
    if (fix && fix.status.status >= 0) {
      setTrail((prev) => {
        const next: [number, number][] = [
          ...prev,
          [fix.latitude, fix.longitude],
        ]
        return next.length > MAX_TRAIL_POINTS ? next.slice(-MAX_TRAIL_POINTS) : next
      })
    }
  }, [fix])

  return { fix, trail }
}
