import { useEffect, useRef, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { RosImage } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

const MAX_DEPTH_MM = 5000
const MAX_DEPTH_M = 5.0

export function useDepthImage(client: FoxgloveClientHandle): string | null {
  const [dataUrl, setDataUrl] = useState<string | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    canvasRef.current = document.createElement("canvas")
    return () => {
      canvasRef.current = null
    }
  }, [])

  useEffect(() => {
    if (client.status !== "connected") return

    const unsub = client.subscribe(
      TOPICS.DEPTH_IMAGE,
      "sensor_msgs/msg/Image",
      (msg) => {
        const image = msg as RosImage
        if (!image.width || !image.height) return

        const canvas = canvasRef.current
        if (!canvas) return
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
        const step = image.step > 0 ? image.step : image.width * 2

        if (image.encoding === "16UC1" || image.encoding === "mono16") {
          for (let y = 0; y < image.height; y++) {
            for (let x = 0; x < image.width; x++) {
              const src = y * step + x * 2
              const lo = data[src] ?? 0
              const hi = data[src + 1] ?? 0
              const depthMm = le ? lo | (hi << 8) : (lo << 8) | hi
              const dst = (y * image.width + x) * 4
              if (depthMm === 0) {
                buf[dst + 3] = 0
              } else {
                const v = Math.max(0, Math.floor(255 * (1 - depthMm / MAX_DEPTH_MM)))
                buf[dst] = v
                buf[dst + 1] = v
                buf[dst + 2] = v
                buf[dst + 3] = 255
              }
            }
          }
        } else if (image.encoding === "32FC1") {
          const floatStep = image.step > 0 ? image.step : image.width * 4
          const rawBuf = data.buffer as ArrayBuffer
          const safeBuf =
            typeof SharedArrayBuffer !== "undefined" && rawBuf instanceof SharedArrayBuffer
              ? rawBuf.slice(0)
              : rawBuf
          const view = new DataView(safeBuf, data.byteOffset, data.byteLength)
          for (let y = 0; y < image.height; y++) {
            for (let x = 0; x < image.width; x++) {
              const src = y * floatStep + x * 4
              if (src + 4 > data.byteLength) break
              const depthM = view.getFloat32(src, le)
              const dst = (y * image.width + x) * 4
              if (!isFinite(depthM) || depthM <= 0) {
                buf[dst + 3] = 0
              } else {
                const v = Math.max(0, Math.floor(255 * (1 - depthM / MAX_DEPTH_M)))
                buf[dst] = v
                buf[dst + 1] = v
                buf[dst + 2] = v
                buf[dst + 3] = 255
              }
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
