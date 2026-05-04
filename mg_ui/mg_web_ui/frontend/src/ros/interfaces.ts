const NODE_NS = {
  WAYPOINT_SEQUENCER: "waypoint_sequencer_node",
  DIAGNOSTICS: "",
  LOCALIZATION: "",
} as const

function nodeNs(ns: string, path: string): string {
  return ns ? `/${ns}${path}` : path
}

export const TOPICS = {
  WAYPOINT_STATUS: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/status"),
  WAYPOINT_PAUSE_REQUEST: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/pause_request"),
  WAYPOINT_SET_NEXT_INDEX: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/set_next_waypoint_index"),
  DIAGNOSTICS: nodeNs(NODE_NS.DIAGNOSTICS, "/diagnostics"),
  INITIALPOSE: nodeNs(NODE_NS.LOCALIZATION, "/initialpose"),
  NAV_ACTION_STATUS: "/navigate_to_pose/_action/status",
  AMCL_POSE: "/amcl_pose",
  MAP: "/map",
  TF: "/tf",
  TF_STATIC: "/tf_static",
  SCAN_TOP: "/scan_top_lidar",
  SCAN_FRONT: "/scan_front_lidar",
  NAV_PLAN: "/plan",
  ACTUAL_PATH: "/actual_path",
  GLOBAL_COSTMAP: "/global_costmap/costmap",
  LOCAL_COSTMAP: "/local_costmap/costmap",
  WAYPOINT_MARKERS: "/waypoint_follower_node/waypoints_markers",
  COLLISION_FRONT: "/collision_detector/polygon_front",
  COLLISION_REAR: "/collision_detector/polygon_rear",
  DEPTH_POINTS: "/camera/camera/depth/color/points",
  CAMERA_IMAGE: "/camera/camera/color/image_raw/compressed",
  PARTICLE_CLOUD: "/particle_cloud",
} as const

export const SERVICES = {
  WAYPOINT_START: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/start"),
  WAYPOINT_STOP: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/stop"),
  WAYPOINT_RELOAD: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, "/reload_waypoints"),
  LIFECYCLE_NAV_IS_ACTIVE: "/lifecycle_manager_navigation/is_active",
  LIFECYCLE_LOC_IS_ACTIVE: "/lifecycle_manager_localization/is_active",
} as const

export interface RosSchema {
  encoding: string
  schemaName: string
  schema: string
}

export const SCHEMAS: Record<string, RosSchema> = {
  'mg_msgs/msg/PauseRequest': {
    encoding: 'cdr',
    schemaName: 'mg_msgs/msg/PauseRequest',
    schema: 'string requester_id\nbool active\nfloat32 heartbeat_period_s\nstring reason',
  },
  'std_msgs/msg/Int16': {
    encoding: 'cdr',
    schemaName: 'std_msgs/msg/Int16',
    schema: 'int16 data',
  },
  'std_msgs/msg/String': {
    encoding: 'cdr',
    schemaName: 'std_msgs/msg/String',
    schema: 'string data',
  },
  'geometry_msgs/msg/PoseWithCovarianceStamped': {
    encoding: 'cdr',
    schemaName: 'geometry_msgs/msg/PoseWithCovarianceStamped',
    schema: `std_msgs/Header header
geometry_msgs/PoseWithCovariance pose
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/PoseWithCovariance
geometry_msgs/Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w`,
  },
}
