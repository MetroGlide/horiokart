export interface Header {
  stamp: { sec: number; nanosec: number }
  frame_id: string
}

export interface Point {
  x: number
  y: number
  z: number
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export interface Pose {
  position: Point
  orientation: Quaternion
}

export interface PoseStamped {
  header: Header
  pose: Pose
}

export interface PoseWithCovarianceStamped {
  header: Header
  pose: {
    pose: Pose
    covariance: number[]
  }
}

export interface OccupancyGrid {
  header: Header
  info: {
    map_load_time: { sec: number; nanosec: number }
    resolution: number
    width: number
    height: number
    origin: Pose
  }
  data: number[] | Int8Array
}

export interface LaserScan {
  header: Header
  angle_min: number
  angle_max: number
  angle_increment: number
  time_increment: number
  scan_time: number
  range_min: number
  range_max: number
  ranges: number[] | Float32Array
  intensities: number[] | Float32Array
}

export interface TransformStamped {
  header: Header
  child_frame_id: string
  transform: {
    translation: Point
    rotation: Quaternion
  }
}

export interface TFMessage {
  transforms: TransformStamped[]
}

export interface Path {
  header: Header
  poses: PoseStamped[]
}

export interface Vector3 {
  x: number
  y: number
  z: number
}

export interface ColorRGBA {
  r: number
  g: number
  b: number
  a: number
}

export const MARKER_TYPE = {
  ARROW: 0,
  CUBE: 1,
  SPHERE: 2,
  CYLINDER: 3,
  LINE_STRIP: 4,
  LINE_LIST: 5,
  CUBE_LIST: 6,
  SPHERE_LIST: 7,
  POINTS: 8,
  TEXT_VIEW_FACING: 9,
} as const

export interface Marker {
  header: Header
  ns: string
  id: number
  type: number
  action: number
  pose: Pose
  scale: Vector3
  color: ColorRGBA
  lifetime: { sec: number; nanosec: number }
  frame_locked: boolean
  points: Point[]
  colors: ColorRGBA[]
  text: string
}

export interface MarkerArray {
  markers: Marker[]
}

export interface Polygon {
  points: Point[]
}

export interface PolygonStamped {
  header: Header
  polygon: Polygon
}

export interface PoseArray {
  header: Header
  poses: Pose[]
}

export interface Particle {
  pose: Pose
  weight: number
}

export interface ParticleCloud {
  header: Header
  particles: Particle[]
}

export interface PointField {
  name: string
  offset: number
  datatype: number
  count: number
}

export interface RosImage {
  header: Header
  height: number
  width: number
  encoding: string
  is_bigendian: boolean
  step: number
  data: Uint8Array | number[]
}

export interface PointCloud2 {
  header: Header
  height: number
  width: number
  fields: PointField[]
  is_bigendian: boolean
  point_step: number
  row_step: number
  data: number[] | Uint8Array
  is_dense: boolean
}

export interface CompressedImage {
  header: Header
  format: string
  data: number[] | Uint8Array
}

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

export interface NavSatStatus {
  status: number
  service: number
}

export interface NavSatFix {
  header: Header
  status: NavSatStatus
  latitude: number
  longitude: number
  altitude: number
  position_covariance: number[]
  position_covariance_type: number
}
