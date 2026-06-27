import { useEffect, useState, useCallback, useRef } from 'react'
import { FoxgloveClientHandle } from './useFoxgloveClient'
import { TOPICS } from '../ros/topics'
import { SERVICES } from '../ros/services'

export interface PoseGraphNode {
  index: number
  x: number
  y: number
  yaw: number
  timestamp: number
}

export interface PoseGraphEdgeEx {
  from: number
  to: number
  score: number
  type: number // 0=ICP, 1=ODOM_FALLBACK (loopの場合は使わないか0)
  info_diag: [number, number, number]
}

export interface GnssPriorEx {
  node_index: number
  sigma_m: number
  status: number
}

export interface GraphStats {
  total_nodes: number
  total_seq_edges: number
  total_loop_edges: number
  icp_attempt_count: number
  icp_success_count: number
  odom_fallback_count: number
  loop_attempt_count: number
  loop_success_count: number
}

export interface PoseGraphState {
  nodes: Map<number, PoseGraphNode>
  seqEdges: Map<string, PoseGraphEdgeEx>
  loopEdges: Map<string, PoseGraphEdgeEx>
  gnssPriors: Map<number, GnssPriorEx>
  stats: GraphStats
}

// Diffメッセージの型
export interface PoseGraphDiffMsg {
  new_node_indices: number[]
  new_node_x: number[]
  new_node_y: number[]
  new_node_yaw: number[]
  new_node_timestamps: number[]
  seq_edge_from: number[]
  seq_edge_to: number[]
  seq_edge_score: number[]
  seq_edge_type: number[]
  seq_edge_info_diag: number[]
  prior_node_indices: number[]
  prior_sigma_m: number[]
  prior_gnss_status: number[]
  loop_edge_from: number[]
  loop_edge_to: number[]
  loop_edge_score: number[]
  loop_edge_info_diag: number[]
  loop_closed: boolean
  full_refresh_needed: boolean
}

// サービスレスポンスの型
export interface GetPoseGraphResponse {
  graph: PoseGraphDiffMsg
  total_nodes: number
  total_seq_edges: number
  total_loop_edges: number
  icp_attempt_count: number
  icp_success_count: number
  odom_fallback_count: number
  loop_attempt_count: number
  loop_success_count: number
}

export function usePoseGraph(client: FoxgloveClientHandle) {
  const [state, setState] = useState<PoseGraphState>({
    nodes: new Map(),
    seqEdges: new Map(),
    loopEdges: new Map(),
    gnssPriors: new Map(),
    stats: {
      total_nodes: 0,
      total_seq_edges: 0,
      total_loop_edges: 0,
      icp_attempt_count: 0,
      icp_success_count: 0,
      odom_fallback_count: 0,
      loop_attempt_count: 0,
      loop_success_count: 0,
    }
  })

  const [isLoading, setIsLoading] = useState(false)
  const isFetchingRef = useRef(false)

  const fetchFullGraph = useCallback(async () => {
    if (client.status !== 'connected' || isFetchingRef.current) return
    isFetchingRef.current = true
    setIsLoading(true)
    try {
      const response = await client.callService(SERVICES.SLAM_GNSS2D_GET_POSE_GRAPH, {}) as GetPoseGraphResponse
      if (!response || !response.graph) return

      const diff = response.graph
      
      const newNodes = new Map<number, PoseGraphNode>()
      for (let i = 0; i < diff.new_node_indices.length; i++) {
        newNodes.set(diff.new_node_indices[i], {
          index: diff.new_node_indices[i],
          x: diff.new_node_x[i],
          y: diff.new_node_y[i],
          yaw: diff.new_node_yaw[i],
          timestamp: diff.new_node_timestamps[i],
        })
      }

      const newSeqEdges = new Map<string, PoseGraphEdgeEx>()
      for (let i = 0; i < diff.seq_edge_from.length; i++) {
        const from = diff.seq_edge_from[i]
        const to = diff.seq_edge_to[i]
        newSeqEdges.set(`${from}-${to}`, {
          from, to,
          score: diff.seq_edge_score[i],
          type: diff.seq_edge_type[i],
          info_diag: [
            diff.seq_edge_info_diag[i * 3],
            diff.seq_edge_info_diag[i * 3 + 1],
            diff.seq_edge_info_diag[i * 3 + 2]
          ]
        })
      }

      const newLoopEdges = new Map<string, PoseGraphEdgeEx>()
      for (let i = 0; i < diff.loop_edge_from.length; i++) {
        const from = diff.loop_edge_from[i]
        const to = diff.loop_edge_to[i]
        newLoopEdges.set(`${from}-${to}`, {
          from, to,
          score: diff.loop_edge_score[i],
          type: 0,
          info_diag: [
            diff.loop_edge_info_diag[i * 3],
            diff.loop_edge_info_diag[i * 3 + 1],
            diff.loop_edge_info_diag[i * 3 + 2]
          ]
        })
      }

      const newPriors = new Map<number, GnssPriorEx>()
      for (let i = 0; i < diff.prior_node_indices.length; i++) {
        const nodeIdx = diff.prior_node_indices[i]
        newPriors.set(nodeIdx, {
          node_index: nodeIdx,
          sigma_m: diff.prior_sigma_m[i],
          status: diff.prior_gnss_status[i],
        })
      }

      setState({
        nodes: newNodes,
        seqEdges: newSeqEdges,
        loopEdges: newLoopEdges,
        gnssPriors: newPriors,
        stats: {
          total_nodes: response.total_nodes,
          total_seq_edges: response.total_seq_edges,
          total_loop_edges: response.total_loop_edges,
          icp_attempt_count: response.icp_attempt_count,
          icp_success_count: response.icp_success_count,
          odom_fallback_count: response.odom_fallback_count,
          loop_attempt_count: response.loop_attempt_count,
          loop_success_count: response.loop_success_count,
        }
      })
    } catch (e) {
      console.error('Failed to fetch pose graph:', e)
    } finally {
      isFetchingRef.current = false
      setIsLoading(false)
    }
  }, [client])

  useEffect(() => {
    if (client.status !== 'connected') return

    const unsub = client.subscribe(TOPICS.SLAM_GNSS2D_POSE_GRAPH_DIFF, 'mg_msgs/msg/PoseGraphDiff', (data) => {
      const diff = data as PoseGraphDiffMsg
      if (diff.full_refresh_needed) {
        fetchFullGraph()
        return
      }

      setState(prev => {
        const nextNodes = new Map(prev.nodes)
        for (let i = 0; i < diff.new_node_indices.length; i++) {
          nextNodes.set(diff.new_node_indices[i], {
            index: diff.new_node_indices[i],
            x: diff.new_node_x[i],
            y: diff.new_node_y[i],
            yaw: diff.new_node_yaw[i],
            timestamp: diff.new_node_timestamps[i],
          })
        }

        const nextSeqEdges = new Map(prev.seqEdges)
        for (let i = 0; i < diff.seq_edge_from.length; i++) {
          const from = diff.seq_edge_from[i]
          const to = diff.seq_edge_to[i]
          nextSeqEdges.set(`${from}-${to}`, {
            from, to,
            score: diff.seq_edge_score[i],
            type: diff.seq_edge_type[i],
            info_diag: [
              diff.seq_edge_info_diag[i * 3],
              diff.seq_edge_info_diag[i * 3 + 1],
              diff.seq_edge_info_diag[i * 3 + 2]
            ]
          })
        }

        const nextLoopEdges = new Map(prev.loopEdges)
        for (let i = 0; i < diff.loop_edge_from.length; i++) {
          const from = diff.loop_edge_from[i]
          const to = diff.loop_edge_to[i]
          nextLoopEdges.set(`${from}-${to}`, {
            from, to,
            score: diff.loop_edge_score[i],
            type: 0,
            info_diag: [
              diff.loop_edge_info_diag[i * 3],
              diff.loop_edge_info_diag[i * 3 + 1],
              diff.loop_edge_info_diag[i * 3 + 2]
            ]
          })
        }

        const nextPriors = new Map(prev.gnssPriors)
        for (let i = 0; i < diff.prior_node_indices.length; i++) {
          const nodeIdx = diff.prior_node_indices[i]
          nextPriors.set(nodeIdx, {
            node_index: nodeIdx,
            sigma_m: diff.prior_sigma_m[i],
            status: diff.prior_gnss_status[i],
          })
        }

        return {
          nodes: nextNodes,
          seqEdges: nextSeqEdges,
          loopEdges: nextLoopEdges,
          gnssPriors: nextPriors,
          stats: {
            ...prev.stats,
            total_nodes: nextNodes.size,
            total_seq_edges: nextSeqEdges.size,
            total_loop_edges: nextLoopEdges.size,
          }
        }
      })
    })

    return () => unsub()
  }, [client, fetchFullGraph])

  // 初回接続時に1度だけフルフェッチを実行する
  useEffect(() => {
    if (client.status === 'connected') {
      fetchFullGraph()
    }
  }, [client.status, fetchFullGraph])

  return { state, isLoading, fetchFullGraph }
}
