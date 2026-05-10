import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useColorImage } from "./hooks/useColorImage";

interface CameraImagePanelProps {
  client: FoxgloveClientHandle;
  className?: string;
}

export default function CameraImagePanel({
  client,
  className,
}: CameraImagePanelProps) {
  const src = useColorImage(client);

  return (
    <div
      className={`flex items-center justify-center bg-black overflow-hidden ${className ?? "w-full h-full"}`}
    >
      {src ? (
        <img
          src={src}
          alt="camera"
          className="max-w-full max-h-full object-contain"
        />
      ) : (
        <span className="text-gray-500 text-xs">No camera image</span>
      )}
    </div>
  );
}
