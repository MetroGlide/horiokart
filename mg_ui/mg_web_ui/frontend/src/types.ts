export interface SequencerStatus {
  stamp: { sec: number; nanosec: number }
  state: string
  current_index: number
  total_waypoints: number
  countdown_ms_remaining: number
  is_paused: boolean
  pause_requesters: string[]
  distance_remaining: number
}

export interface DiagnosticKeyValue {
  key: string
  value: string
}

export interface DiagnosticStatus {
  level: number
  name: string
  message: string
  hardware_id: string
  values: DiagnosticKeyValue[]
}

export interface DiagnosticArray {
  header: { stamp: { sec: number; nanosec: number }; frame_id: string }
  status: DiagnosticStatus[]
}

export const DIAG_LEVEL: Record<number, string> = {
  0: 'OK',
  1: 'WARN',
  2: 'ERROR',
  3: 'STALE',
}

export const DIAG_COLOR: Record<number, string> = {
  0: 'text-green-500',
  1: 'text-yellow-500',
  2: 'text-red-500',
  3: 'text-gray-400',
}
