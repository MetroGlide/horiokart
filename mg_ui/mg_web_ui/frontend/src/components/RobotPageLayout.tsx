import { FoxgloveClientHandle } from "../hooks/useFoxgloveClient";
import { useVisualization } from "../contexts/VisualizationContext";
import { AccordionItem } from "./SideAccordion";
import SideAccordion from "./SideAccordion";
import {
  RosViewer,
  CameraImagePanel,
  DepthImagePanel,
  ViewerMode,
} from "./ros-viewer";
import { ReactNode } from "react";

interface RobotPageLayoutProps {
  client: FoxgloveClientHandle;
  accordionItems: AccordionItem[];
  defaultOpen?: string[];
  viewerMode?: ViewerMode;
  showCameraPanel?: boolean;
  viewerOverlay?: ReactNode;
}

export default function RobotPageLayout({
  client,
  accordionItems,
  defaultOpen = [],
  viewerMode,
  showCameraPanel = false,
  viewerOverlay,
}: RobotPageLayoutProps) {
  const { enabled: vizEnabled, layers } = useVisualization();
  const hasViewer = vizEnabled && viewerMode != null;

  return (
    <div className="flex gap-4 h-[calc(100vh-160px)]">
      <div
        className={
          hasViewer
            ? "w-72 flex-shrink-0 overflow-y-auto"
            : "flex-1 overflow-y-auto"
        }
      >
        <SideAccordion defaultOpen={defaultOpen} items={accordionItems} />
      </div>

      {hasViewer && (
        <div className="flex-1 flex flex-col gap-2 min-h-0">
          <div className="flex-1 min-h-0 bg-gray-900 rounded-lg overflow-hidden border border-gray-700 relative">
            <RosViewer
              client={client}
              initialMode={viewerMode}
              className="w-full h-full"
            />
            {viewerOverlay && (
              <div className="absolute inset-0 pointer-events-none">
                {viewerOverlay}
              </div>
            )}
          </div>
          {showCameraPanel && layers.cameraImage && (
            <div className="flex gap-2 h-40 flex-shrink-0">
              <div className="flex-1 rounded-lg overflow-hidden border border-gray-700">
                <CameraImagePanel client={client} className="w-full h-full" />
              </div>
              <div className="flex-1 rounded-lg overflow-hidden border border-gray-700">
                <DepthImagePanel client={client} className="w-full h-full" />
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
}
