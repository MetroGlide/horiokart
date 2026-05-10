import { useEffect, useRef, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { RosImage } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useColorImage(client: FoxgloveClientHandle): string | null {
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
      TOPICS.CAMERA_IMAGE,
      "sensor_msgs/msg/Image",
      (msg) => {
        const image = msg as RosImage
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

        if (image.encoding === "rgb8") {
          for (let i = 0; i < image.width * image.height; i++) {
            buf[i * 4] = data[i * 3]
            buf[i * 4 + 1] = data[i * 3 + 1]
            buf[i * 4 + 2] = data[i * 3 + 2]
            buf[i * 4 + 3] = 255
          }
        } else if (image.encoding === "bgr8") {
          for (let i = 0; i < image.width * image.height; i++) {
            buf[i * 4] = data[i * 3 + 2]
            buf[i * 4 + 1] = data[i * 3 + 1]
            buf[i * 4 + 2] = data[i * 3]
            buf[i * 4 + 3] = 255
          }
        } else if (image.encoding === "rgba8") {
          buf.set(data.subarray(0, image.width * image.height * 4))
        } else if (image.encoding === "bgra8") {
          for (let i = 0; i < image.width * image.height; i++) {
            buf[i * 4] = data[i * 4 + 2]
            buf[i * 4 + 1] = data[i * 4 + 1]
            buf[i * 4 + 2] = data[i * 4]
            buf[i * 4 + 3] = data[i * 4 + 3]
          }
        }

        ctx.putImageData(imageData, 0, 0)
        setDataUrl(canvas.toDataURL("image/jpeg", 0.85))
      },
    )

    return () => {
      unsub()
      setDataUrl(null)
    }
  }, [client, client.status])

  return dataUrl
}
