import { useEffect, useState } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'

export function useTopicSubscriber<T>(
  client: FoxgloveClientHandle,
  topic: string,
  schemaName: string,
): T | null {
  const [data, setData] = useState<T | null>(null)

  useEffect(() => {
    const unsubscribe = client.subscribe(topic, schemaName, (msg) => {
      setData(msg as T)
    })
    return unsubscribe
  }, [client.status, client.subscribe, topic, schemaName])

  return data
}
