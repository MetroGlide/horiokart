import { useCallback, useEffect, useRef, useState } from 'react'
import type { ApiLog, CallApi, SystemManagerHandle } from '../types/api'
import { getSysManagerUrl } from '../utils/systemManagerConfig'

export type { ApiLog, CallApi, SystemManagerHandle }

export function useSystemManagerClient(): SystemManagerHandle {
  const [containers, setContainers] = useState<Record<string, string>>({})
  const [logs, setLogs] = useState<ApiLog[]>([])
  const logIdRef = useRef(0)

  useEffect(() => {
    const poll = () =>
      fetch(`${getSysManagerUrl()}/status`)
        .then((r) => r.json())
        .then(setContainers)
        .catch(() => {})
    poll()
    const id = setInterval(poll, 2000)
    return () => clearInterval(id)
  }, [])

  const callApi: CallApi = useCallback(async (path, body) => {
    const r = await fetch(`${getSysManagerUrl()}${path}`, {
      method: 'POST',
      headers: body !== undefined ? { 'Content-Type': 'application/json' } : {},
      body: body !== undefined ? JSON.stringify(body) : undefined,
    })
    const data = await r.json()
    const entry: ApiLog = {
      id: ++logIdRef.current,
      timestamp: new Date().toLocaleTimeString(),
      path,
      success: data.success ?? true,
      message: data.message ?? '',
    }
    setLogs((prev) => [entry, ...prev].slice(0, 50))
    return data
  }, [])

  return { containers, logs, callApi }
}
