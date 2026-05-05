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

export interface GoalStatus {
  goal_info: {
    goal_id: { uuid: number[] }
    stamp: { sec: number; nanosec: number }
  }
  status: number
}

export interface GoalStatusArray {
  status_list: GoalStatus[]
}

export const GOAL_STATUS: Record<number, string> = {
  0: 'UNKNOWN',
  1: 'ACCEPTED',
  2: 'EXECUTING',
  3: 'CANCELING',
  4: 'SUCCEEDED',
  5: 'CANCELED',
  6: 'ABORTED',
}

export const GOAL_STATUS_COLOR: Record<number, string> = {
  0: 'text-gray-400',
  1: 'text-yellow-400',
  2: 'text-blue-400',
  3: 'text-yellow-500',
  4: 'text-green-400',
  5: 'text-gray-400',
  6: 'text-red-400',
}

export interface Float32Msg {
  data: number
}

export interface BoolMsg {
  data: boolean
}

export interface OdomMsg {
  twist: {
    twist: {
      linear: { x: number; y: number; z: number }
      angular: { x: number; y: number; z: number }
    }
  }
}

export interface CollisionDetectorState {
  polygons: string[]
  detections: boolean[]
}

export interface TwistMsg {
  linear: { x: number; y: number; z: number }
  angular: { x: number; y: number; z: number }
}
