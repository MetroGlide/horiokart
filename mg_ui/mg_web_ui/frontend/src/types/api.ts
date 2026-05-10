export type ContainerStatus = 'running' | 'exited' | 'dead' | 'unknown'

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

export interface LogEntry {
  id: number
  service: string
  line: string
}

export interface SystemManagerHandle {
  containers: Record<string, string>
  logs: ApiLog[]
  callApi: CallApi
}
