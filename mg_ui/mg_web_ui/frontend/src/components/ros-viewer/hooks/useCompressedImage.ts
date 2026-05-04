import { useEffect, useState } from "react"
import { FoxgloveClientHandle } from "../../../hooks/useFoxgloveClient"
import { CompressedImage } from "../../../types/ros-types"
import { TOPICS } from "../../../ros/interfaces"

export function useCompressedImage(client: FoxgloveClientHandle): string | null {
  const [objectUrl, setObjectUrl] = useState<string | null>(null)

  useEffect(() => {
    if (client.status !== "connected") return

    let currentUrl: string | null = null

    const unsub = client.subscribe(
      TOPICS.CAMERA_IMAGE,
      "sensor_msgs/msg/CompressedImage",
      (msg) => {
        const image = msg as CompressedImage
        const src =
          image.data instanceof Uint8Array
            ? new Uint8Array(image.data.buffer as ArrayBuffer)
            : new Uint8Array(image.data as number[])
        const blob = new Blob([src], { type: `image/${image.format}` })
        const url = URL.createObjectURL(blob)

        setObjectUrl((prev) => {
          if (prev) URL.revokeObjectURL(prev)
          return url
        })
        currentUrl = url
      },
    )

    return () => {
      unsub()
      if (currentUrl) URL.revokeObjectURL(currentUrl)
    }
  }, [client, client.status])

  return objectUrl
}
