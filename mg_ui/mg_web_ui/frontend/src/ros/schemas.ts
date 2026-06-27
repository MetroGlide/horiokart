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
  'geometry_msgs/msg/Twist': {
    encoding: 'cdr',
    schemaName: 'geometry_msgs/msg/Twist',
    schema: `geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z`,
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
  'geometry_msgs/msg/PoseStamped': {
    encoding: 'cdr',
    schemaName: 'geometry_msgs/msg/PoseStamped',
    schema: `std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
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
  'mg_msgs/msg/PoseGraphDiff': {
    encoding: 'cdr',
    schemaName: 'mg_msgs/msg/PoseGraphDiff',
    schema: `int32[] new_node_indices
float64[] new_node_x
float64[] new_node_y
float64[] new_node_yaw
float64[] new_node_timestamps
int32[] seq_edge_from
int32[] seq_edge_to
float64[] seq_edge_score
int32[] seq_edge_type
float64[] seq_edge_info_diag
int32[] prior_node_indices
float64[] prior_sigma_m
int32[] prior_gnss_status
int32[] loop_edge_from
int32[] loop_edge_to
float64[] loop_edge_score
float64[] loop_edge_info_diag
bool loop_closed
bool full_refresh_needed`,
  },
}
