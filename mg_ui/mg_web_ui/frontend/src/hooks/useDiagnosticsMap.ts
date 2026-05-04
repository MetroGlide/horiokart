import { useEffect, useState } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'
import { DiagnosticArray, DiagnosticStatus } from '../types'

export function useDiagnosticsMap(
  client: FoxgloveClientHandle,
): DiagnosticStatus[] {
  const [map, setMap] = useState<Map<string, DiagnosticStatus>>(new Map())

  useEffect(() => {
    if (client.status !== 'connected') return
    const unsubscribe = client.subscribe(
      '/diagnostics',
      'diagnostic_msgs/msg/DiagnosticArray',
      (msg) => {
        const arr = msg as DiagnosticArray
        setMap((prev) => {
          const next = new Map(prev)
          arr.status.forEach((s) => next.set(s.name, s))
          return next
        })
      },
    )
    return unsubscribe
  }, [client, client.status])

  return Array.from(map.values()).sort((a, b) => a.name.localeCompare(b.name))
}
