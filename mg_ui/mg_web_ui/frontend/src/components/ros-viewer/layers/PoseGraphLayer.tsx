import React, { useMemo, useRef } from 'react'
import * as THREE from 'three'
import { useFrame } from '@react-three/fiber'
import { PoseGraphState, PoseGraphNode, PoseGraphEdgeEx, GnssPriorEx } from '../../../hooks/usePoseGraph'

interface PoseGraphLayerProps {
  state: PoseGraphState
  showNodes?: boolean
  showSeqEdges?: boolean
  showLoopEdges?: boolean
  showGnssPriors?: boolean
  onNodeClick?: (nodeId: number) => void
}

export function PoseGraphLayer({ 
  state, 
  showNodes = true, 
  showSeqEdges = true, 
  showLoopEdges = true, 
  showGnssPriors = true, 
  onNodeClick 
}: PoseGraphLayerProps) {
  const nodeMeshRef = useRef<THREE.InstancedMesh>(null)
  
  const nodeGeometry = useMemo(() => new THREE.SphereGeometry(0.15, 8, 8), [])
  const nodeMaterial = useMemo(() => new THREE.MeshBasicMaterial({ color: 0x00ffff }), [])
  
  useFrame(() => {
    if (!nodeMeshRef.current || state.nodes.size === 0) return
    const mesh = nodeMeshRef.current
    const nodes = Array.from(state.nodes.values())
    
    mesh.count = nodes.length
    
    const dummy = new THREE.Object3D()
    const color = new THREE.Color()
    
    nodes.forEach((node: PoseGraphNode, i: number) => {
      dummy.position.set(node.x, node.y, 0)
      dummy.rotation.z = node.yaw
      dummy.updateMatrix()
      mesh.setMatrixAt(i, dummy.matrix)
      
      if (i === nodes.length - 1) {
        color.setHex(0xff0000)
      } else {
        color.setHex(0x00ffff)
      }
      mesh.setColorAt(i, color)
    })
    
    if (mesh.instanceMatrix) mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
  })

  const seqEdgePositions = useMemo(() => {
    const arr: number[] = []
    state.seqEdges.forEach((edge: PoseGraphEdgeEx) => {
      const fromNode = state.nodes.get(edge.from)
      const toNode = state.nodes.get(edge.to)
      if (fromNode && toNode) {
        arr.push(fromNode.x, fromNode.y, 0)
        arr.push(toNode.x, toNode.y, 0)
      }
    })
    return new Float32Array(arr)
  }, [state.seqEdges, state.nodes])

  const loopEdgePositions = useMemo(() => {
    const arr: number[] = []
    state.loopEdges.forEach((edge: PoseGraphEdgeEx) => {
      const fromNode = state.nodes.get(edge.from)
      const toNode = state.nodes.get(edge.to)
      if (fromNode && toNode) {
        arr.push(fromNode.x, fromNode.y, 0.05)
        arr.push(toNode.x, toNode.y, 0.05)
      }
    })
    return new Float32Array(arr)
  }, [state.loopEdges, state.nodes])

  return (
    <group>
      {showNodes && state.nodes.size > 0 && (
        <instancedMesh
          ref={nodeMeshRef}
          args={[nodeGeometry, nodeMaterial, Math.max(state.nodes.size, 100)]}
          onClick={(e) => {
            if (e.instanceId !== undefined && onNodeClick) {
              const nodesArray = Array.from(state.nodes.values()) as PoseGraphNode[]
              if (nodesArray[e.instanceId]) {
                onNodeClick(nodesArray[e.instanceId].index)
              }
            }
          }}
        />
      )}
      
      {showSeqEdges && seqEdgePositions.length > 0 && (
        <lineSegments>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={seqEdgePositions.length / 3}
              array={seqEdgePositions}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color={0x00ff00} opacity={0.6} transparent />
        </lineSegments>
      )}

      {showLoopEdges && loopEdgePositions.length > 0 && (
        <lineSegments>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={loopEdgePositions.length / 3}
              array={loopEdgePositions}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color={0xff00ff} linewidth={2} />
        </lineSegments>
      )}

      {showGnssPriors && Array.from(state.gnssPriors.values()).map((prior: GnssPriorEx) => {
        const node = state.nodes.get(prior.node_index)
        if (!node) return null
        return (
          <mesh key={`prior-${prior.node_index}`} position={[node.x, node.y, 0.1]}>
            <circleGeometry args={[0.3, 4]} />
            <meshBasicMaterial color={0xffaa00} wireframe />
          </mesh>
        )
      })}
    </group>
  )
}
