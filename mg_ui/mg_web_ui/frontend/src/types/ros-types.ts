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
