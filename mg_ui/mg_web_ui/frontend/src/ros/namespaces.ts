export const NODE_NS = {
  WAYPOINT_SEQUENCER: 'waypoint_sequencer_node',
  DIAGNOSTICS: '',
  LOCALIZATION: '',
} as const

export function nodeNs(ns: string, path: string): string {
  return ns ? `/${ns}${path}` : path
}
