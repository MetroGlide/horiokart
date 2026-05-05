import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useDepthImage } from "./hooks/useDepthImage";

interface DepthImagePanelProps {
  client: FoxgloveClientHandle;
  className?: string;
}

export default function DepthImagePanel({
  client,
  className,
}: DepthImagePanelProps) {
  const src = useDepthImage(client);

  return (
    <div
      className={`flex items-center justify-center bg-black overflow-hidden ${className ?? "w-full h-full"}`}
    >
      {src ? (
        <img
          src={src}
          alt="depth"
          className="max-w-full max-h-full object-contain"
        />
      ) : (
        <span className="text-gray-500 text-xs">No depth image</span>
      )}
    </div>
  );
}
