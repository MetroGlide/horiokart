import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import { AccordionItem } from "./SideAccordion";
import SideAccordion from "./SideAccordion";
import {
  RosViewer,
  CameraImagePanel,
  DepthImagePanel,
  ViewerMode,
  ViewerInteractionMode,
} from "../ros-viewer";
import { ReactNode, useState } from "react";
import VelocityGauge from "../panels/VelocityGauge";
import SystemMetrics from "../panels/SystemMetrics";
import JoystickPad from "../panels/JoystickPad";

interface RobotPageLayoutProps {
  client: FoxgloveClientHandle;
  accordionItems: AccordionItem[];
  defaultOpen?: string[];
  viewerMode?: ViewerMode;
  interactionMode?: ViewerInteractionMode;
  onPoseSet?: (x: number, y: number, yaw: number) => void;
  extraPanels?: ReactNode;
  extraOverlay?: ReactNode;
}

export default function RobotPageLayout({
  client,
  accordionItems,
  defaultOpen = [],
  viewerMode,
  interactionMode,
  onPoseSet,
  extraPanels,
  extraOverlay,
}: RobotPageLayoutProps) {
  const { enabled: vizEnabled, layers, overlays } = useVisualization();
  const hasViewer = vizEnabled && viewerMode != null;

  const [sidebarCollapsed, setSidebarCollapsed] = useState(false);
  const [viewerCollapsed, setViewerCollapsed] = useState(false);

  const collapseButton = (
    onClick: () => void,
    icon: ReactNode,
    position: "left" | "right",
  ) => (
    <button
      onClick={onClick}
      className={`flex-shrink-0 w-5 self-stretch flex items-center justify-center bg-gray-800 hover:bg-gray-700 transition-colors rounded-lg text-gray-400 hover:text-white ${
        position === "left" ? "rounded-l-lg" : "rounded-r-lg"
      }`}
      title={position === "left" ? "サイドバーを展開" : "ビューワーを展開"}
    >
      {icon}
    </button>
  );

  const chevronLeft = (
    <svg
      className="w-3 h-3"
      fill="none"
      viewBox="0 0 24 24"
      stroke="currentColor"
    >
      <path
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeWidth={2}
        d="M15 19l-7-7 7-7"
      />
    </svg>
  );
  const chevronRight = (
    <svg
      className="w-3 h-3"
      fill="none"
      viewBox="0 0 24 24"
      stroke="currentColor"
    >
      <path
        strokeLinecap="round"
        strokeLinejoin="round"
        strokeWidth={2}
        d="M9 5l7 7-7 7"
      />
    </svg>
  );

  return (
    <div className="flex gap-2 h-[calc(100vh-160px)]">
      {/* サイドバー */}
      {sidebarCollapsed ? (
        collapseButton(() => setSidebarCollapsed(false), chevronRight, "left")
      ) : (
        <div
          className={`flex gap-2 ${hasViewer && !viewerCollapsed ? "w-72 flex-shrink-0" : "flex-1"}`}
        >
          <div className="flex-1 overflow-y-auto">
            <SideAccordion defaultOpen={defaultOpen} items={accordionItems} />
          </div>
          {hasViewer && (
            <button
              onClick={() => setSidebarCollapsed(true)}
              className="flex-shrink-0 w-5 self-stretch flex items-center justify-center bg-gray-800 hover:bg-gray-700 transition-colors rounded-lg text-gray-400 hover:text-white"
              title="サイドバーを最小化"
            >
              {chevronLeft}
            </button>
          )}
        </div>
      )}

      {/* ビューワー */}
      {hasViewer &&
        (viewerCollapsed ? (
          collapseButton(() => setViewerCollapsed(false), chevronLeft, "right")
        ) : (
          <div className="flex-1 flex gap-2 min-h-0">
            <button
              onClick={() => setViewerCollapsed(true)}
              className="flex-shrink-0 w-5 self-stretch flex items-center justify-center bg-gray-800 hover:bg-gray-700 transition-colors rounded-lg text-gray-400 hover:text-white"
              title="ビューワーを最小化"
            >
              {chevronRight}
            </button>
            <div className="flex-1 flex flex-col gap-2 min-h-0">
              <div className="flex-1 min-h-0 bg-gray-900 rounded-lg overflow-hidden border border-gray-700 relative">
                <RosViewer
                  client={client}
                  initialMode={viewerMode}
                  interactionMode={interactionMode}
                  onPoseSet={onPoseSet}
                  className="w-full h-full"
                />
                <div className="absolute inset-0 pointer-events-none">
                  <div className="absolute top-10 left-2 flex flex-col gap-2 pointer-events-auto w-40">
                    {overlays.velocityGauge && (
                      <VelocityGauge client={client} compact />
                    )}
                    {overlays.systemMetrics && (
                      <SystemMetrics client={client} compact />
                    )}
                  </div>
                  {overlays.joystick && (
                    <div className="absolute bottom-4 right-4 pointer-events-auto">
                      <JoystickPad client={client} />
                    </div>
                  )}
                  {extraOverlay}
                </div>
              </div>
              {(layers.colorImage || layers.depthImage) && (
                <div className="flex gap-2 h-40 flex-shrink-0">
                  {layers.colorImage && (
                    <div className="flex-1 rounded-lg overflow-hidden border border-gray-700">
                      <CameraImagePanel
                        client={client}
                        className="w-full h-full"
                      />
                    </div>
                  )}
                  {layers.depthImage && (
                    <div className="flex-1 rounded-lg overflow-hidden border border-gray-700">
                      <DepthImagePanel
                        client={client}
                        className="w-full h-full"
                      />
                    </div>
                  )}
                </div>
              )}
              {extraPanels}
            </div>
          </div>
        ))}
    </div>
  );
}
