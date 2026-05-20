import { useEffect, useRef, useCallback } from "react"
import * as THREE from "three"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { TFMessage, TransformStamped } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export interface TfBuffer {
  lookupTransform: (targetFrame: string, sourceFrame: string) => THREE.Matrix4 | null
}

export function useTfBuffer(client: FoxgloveClientHandle): TfBuffer {
  // child_frame_id → TransformStamped (latest). Updated via ref to avoid re-renders.
  const transformsRef = useRef<Map<string, TransformStamped>>(new Map())

  useEffect(() => {
    if (client.status !== "connected") return

    const applyTf = (msg: unknown) => {
      const tf = msg as TFMessage
      for (const t of tf.transforms) {
        transformsRef.current.set(t.child_frame_id, t)
      }
    }

    const unsubTf = client.subscribe(TOPICS.TF, "tf2_msgs/msg/TFMessage", applyTf)
    const unsubStatic = client.subscribe(TOPICS.TF_STATIC, "tf2_msgs/msg/TFMessage", applyTf)
    return () => {
      unsubTf()
      unsubStatic()
    }
  }, [client.status])

  // BFS from sourceFrame up through parent links to reach targetFrame.
  // Each TransformStamped with child_frame_id=C represents the transform from C to its parent P:
  //   p_P = M_P_C * p_C   (M_P_C = compose(translation, rotation))
  // Accumulated matrix M satisfies: p_currentFrame = M * p_source
  // When currentFrame === targetFrame, M is the result.
  const lookupTransform = useCallback(
    (targetFrame: string, sourceFrame: string): THREE.Matrix4 | null => {
      if (sourceFrame === targetFrame) return new THREE.Matrix4()

      const visited = new Set<string>()
      const queue: Array<{ frame: string; matrix: THREE.Matrix4 }> = [
        { frame: sourceFrame, matrix: new THREE.Matrix4() },
      ]

      while (queue.length > 0) {
        const { frame, matrix } = queue.shift()!
        if (visited.has(frame)) continue
        visited.add(frame)

        if (frame === targetFrame) return matrix

        const ts = transformsRef.current.get(frame)
        if (!ts) continue

        const { translation: t, rotation: r } = ts.transform
        const M_parent_child = new THREE.Matrix4().compose(
          new THREE.Vector3(t.x, t.y, t.z),
          new THREE.Quaternion(r.x, r.y, r.z, r.w),
          new THREE.Vector3(1, 1, 1),
        )
        const nextMatrix = new THREE.Matrix4().multiplyMatrices(M_parent_child, matrix)
        queue.push({ frame: ts.header.frame_id, matrix: nextMatrix })
      }

      return null
    },
    [],
  )

  return { lookupTransform }
}
