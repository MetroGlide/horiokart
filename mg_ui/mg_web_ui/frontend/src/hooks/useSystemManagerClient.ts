import { useCallback, useEffect, useRef, useState } from 'react'

export type CallApi = (
  path: string,
  body?: unknown,
) => Promise<{ success: boolean; message: string }>

export interface ApiLog {
  id: number
  timestamp: string
  path: string
  success: boolean
  message: string
}

export interface SystemManagerHandle {
  containers: Record<string, string>
  logs: ApiLog[]
  callApi: CallApi
}

const BASE_URL = `http://${window.location.hostname}:8001`

export function useSystemManagerClient(): SystemManagerHandle {
  const [containers, setContainers] = useState<Record<string, string>>({})
  const [logs, setLogs] = useState<ApiLog[]>([])
  const logIdRef = useRef(0)

  useEffect(() => {
    const poll = () =>
      fetch(`${BASE_URL}/status`)
        .then((r) => r.json())
        .then(setContainers)
        .catch(() => {})
    poll()
    const id = setInterval(poll, 2000)
    return () => clearInterval(id)
  }, [])

  const callApi: CallApi = useCallback(async (path, body) => {
    const r = await fetch(`${BASE_URL}${path}`, {
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
