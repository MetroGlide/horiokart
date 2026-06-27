import React from 'react'
import { PoseGraphState } from '../../hooks/usePoseGraph'

interface Props {
  state: PoseGraphState
  selectedNodeIndex: number | null
  onClose: () => void
}

export function PoseGraphDetailPanel({ state, selectedNodeIndex, onClose }: Props) {
  if (selectedNodeIndex === null) return null
  
  const node = state.nodes.get(selectedNodeIndex)
  if (!node) return null

  const prior = state.gnssPriors.get(selectedNodeIndex)
  
  const relatedEdges = Array.from(state.seqEdges.values()).filter(
    e => e.from === selectedNodeIndex || e.to === selectedNodeIndex
  )
  const relatedLoopEdges = Array.from(state.loopEdges.values()).filter(
    e => e.from === selectedNodeIndex || e.to === selectedNodeIndex
  )

  return (
    <div className="absolute top-4 left-4 w-80 bg-gray-900/90 text-white rounded-lg border border-gray-700 shadow-xl overflow-hidden flex flex-col z-50 backdrop-blur-md">
      <div className="flex justify-between items-center px-4 py-2 bg-gray-800 border-b border-gray-700">
        <h3 className="font-semibold text-sm">Node #{selectedNodeIndex} Details</h3>
        <button onClick={onClose} className="text-gray-400 hover:text-white">✕</button>
      </div>
      
      <div className="p-4 space-y-4 overflow-y-auto max-h-[80vh] text-sm custom-scrollbar">
        <section>
          <h4 className="text-gray-400 text-xs font-semibold uppercase mb-1">Pose</h4>
          <div className="grid grid-cols-2 gap-1 font-mono text-gray-200">
            <span>X:</span><span>{node.x.toFixed(3)} m</span>
            <span>Y:</span><span>{node.y.toFixed(3)} m</span>
            <span>Yaw:</span><span>{node.yaw.toFixed(3)} rad</span>
            <span>Time:</span><span>{node.timestamp.toFixed(3)}</span>
          </div>
        </section>

        {prior && (
          <section>
            <h4 className="text-orange-400 text-xs font-semibold uppercase mb-1">GNSS Prior</h4>
            <div className="grid grid-cols-2 gap-1 font-mono text-gray-200 bg-orange-900/20 p-2 rounded">
              <span>Sigma:</span><span>{prior.sigma_m > 0 ? `${prior.sigma_m.toFixed(3)} m` : 'N/A'}</span>
              <span>Status:</span><span>{prior.status}</span>
            </div>
          </section>
        )}

        <section>
          <h4 className="text-green-400 text-xs font-semibold uppercase mb-1">Seq Edges ({relatedEdges.length})</h4>
          <ul className="space-y-2">
            {relatedEdges.map(e => (
              <li key={`${e.from}-${e.to}`} className="bg-gray-800/80 p-2 rounded">
                <div className="font-mono text-xs">{e.from} → {e.to}</div>
                <div className="grid grid-cols-2 gap-x-2 text-xs text-gray-300 mt-1">
                  <span>Score:</span><span className={e.score > 0.05 ? "text-red-400" : ""}>{e.score > 0 ? e.score.toFixed(4) : "N/A"}</span>
                  <span>Type:</span><span className={e.type === 1 ? "text-yellow-400" : ""}>{e.type === 1 ? 'OdomFallback' : 'ICP'}</span>
                </div>
              </li>
            ))}
          </ul>
        </section>

        {relatedLoopEdges.length > 0 && (
          <section>
            <h4 className="text-fuchsia-400 text-xs font-semibold uppercase mb-1">Loop Edges ({relatedLoopEdges.length})</h4>
            <ul className="space-y-2">
              {relatedLoopEdges.map(e => (
                <li key={`loop-${e.from}-${e.to}`} className="bg-fuchsia-900/20 p-2 rounded border border-fuchsia-500/30">
                  <div className="font-mono text-xs text-fuchsia-200">{e.from} → {e.to}</div>
                  <div className="grid grid-cols-2 gap-x-2 text-xs text-gray-300 mt-1">
                    <span>Score:</span><span>{e.score > 0 ? e.score.toFixed(4) : "N/A"}</span>
                  </div>
                </li>
              ))}
            </ul>
          </section>
        )}

        <section className="pt-2 border-t border-gray-700">
          <h4 className="text-gray-400 text-xs font-semibold uppercase mb-1 flex justify-between">
            <span>Graph Stats</span>
          </h4>
          <div className="grid grid-cols-2 gap-1 font-mono text-xs text-gray-300 bg-gray-800/50 p-2 rounded">
            <span>Nodes:</span><span>{state.stats.total_nodes}</span>
            <span>Seq Edges:</span><span>{state.stats.total_seq_edges}</span>
            <span>Loop Edges:</span><span>{state.stats.total_loop_edges}</span>
            <span>ICP Success:</span>
            <span className={state.stats.icp_attempt_count > 0 && (state.stats.icp_success_count/state.stats.icp_attempt_count) < 0.9 ? 'text-red-400' : 'text-green-400'}>
              {state.stats.icp_attempt_count > 0 ? `${(state.stats.icp_success_count/state.stats.icp_attempt_count*100).toFixed(1)}%` : 'N/A'}
            </span>
            <span>Odom FB:</span><span className={state.stats.odom_fallback_count > 0 ? 'text-yellow-400' : ''}>{state.stats.odom_fallback_count}</span>
          </div>
        </section>
      </div>
    </div>
  )
}
