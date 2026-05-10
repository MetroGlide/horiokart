import { useCallback, useState } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'

export interface ServiceCallState {
  loading: boolean
  error: string | null
}

export function useServiceCaller(client: FoxgloveClientHandle) {
  const [state, setState] = useState<ServiceCallState>({ loading: false, error: null })

  const call = useCallback(
    async (service: string, payload: unknown = {}): Promise<unknown> => {
      setState({ loading: true, error: null })
      try {
        const result = await client.callService(service, payload)
        setState({ loading: false, error: null })
        return result
      } catch (e) {
        const msg = e instanceof Error ? e.message : String(e)
        setState({ loading: false, error: msg })
        throw e
      }
    },
    [client],
  )

  return { call, ...state }
}
