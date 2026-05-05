import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { RosImage } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

const MAX_DEPTH_MM = 5000
const MAX_DEPTH_M = 5.0

export function useDepthImage(client: FoxgloveClientHandle): string | null {
  const [dataUrl, setDataUrl] = useState<string | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return

    const unsub = client.subscribe(
      TOPICS.DEPTH_IMAGE,
      "sensor_msgs/msg/Image",
      (msg) => {
        const image = msg as RosImage
        const canvas = document.createElement("canvas")
        canvas.width = image.width
        canvas.height = image.height
        const ctx = canvas.getContext("2d")
        if (!ctx) return

        const imageData = ctx.createImageData(image.width, image.height)
        const buf = imageData.data
        const data =
          image.data instanceof Uint8Array
            ? image.data
            : new Uint8Array(image.data as number[])
        const le = !image.is_bigendian

        if (image.encoding === "16UC1") {
          for (let i = 0; i < image.width * image.height; i++) {
            const lo = data[i * 2]
            const hi = data[i * 2 + 1]
            const depthMm = le ? lo | (hi << 8) : (lo << 8) | hi
            const idx = i * 4
            if (depthMm === 0) {
              buf[idx + 3] = 0
            } else {
              const v = Math.max(0, Math.floor(255 * (1 - depthMm / MAX_DEPTH_MM)))
              buf[idx] = v
              buf[idx + 1] = v
              buf[idx + 2] = v
              buf[idx + 3] = 255
            }
          }
        } else if (image.encoding === "32FC1") {
          const view = new DataView(
            data.buffer instanceof SharedArrayBuffer
              ? data.buffer.slice(0)
              : (data.buffer as ArrayBuffer),
            data.byteOffset,
            data.byteLength,
          )
          for (let i = 0; i < image.width * image.height; i++) {
            const depthM = view.getFloat32(i * 4, le)
            const idx = i * 4
            if (!isFinite(depthM) || depthM <= 0) {
              buf[idx + 3] = 0
            } else {
              const v = Math.max(0, Math.floor(255 * (1 - depthM / MAX_DEPTH_M)))
              buf[idx] = v
              buf[idx + 1] = v
              buf[idx + 2] = v
              buf[idx + 3] = 255
            }
          }
        }

        ctx.putImageData(imageData, 0, 0)
        setDataUrl(canvas.toDataURL("image/png"))
      },
    )

    return () => {
      unsub()
      setDataUrl(null)
    }
  }, [client, client.status])

  return dataUrl
}
