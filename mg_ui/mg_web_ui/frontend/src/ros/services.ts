import { NODE_NS, nodeNs } from './namespaces'

export const SERVICES = {
  WAYPOINT_START: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/start'),
  WAYPOINT_STOP: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/stop'),
  WAYPOINT_RELOAD: nodeNs(NODE_NS.WAYPOINT_SEQUENCER, '/reload_waypoints'),
  LIFECYCLE_NAV_IS_ACTIVE: '/lifecycle_manager_navigation/is_active',
  LIFECYCLE_LOC_IS_ACTIVE: '/lifecycle_manager_localization/is_active',
  ROSBAG_PAUSE: '/rosbag2_player/pause',
  ROSBAG_RESUME: '/rosbag2_player/resume',
  ROSBAG_SET_RATE: '/rosbag2_player/set_rate',
} as const
