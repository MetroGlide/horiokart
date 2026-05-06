import { FoxgloveClientHandle } from "../../hooks/useFoxgloveClient";
import { useVisualization } from "../../contexts/VisualizationContext";
import { AccordionItem } from "./SideAccordion";
import SideAccordion from "./SideAccordion";
import {
  RosViewer,
  CameraImagePanel,
  DepthImagePanel,
  ViewerMode,
} from "../ros-viewer";
import { ReactNode } from "react";
import VelocityGauge from "../panels/VelocityGauge";
import SystemMetrics from "../panels/SystemMetrics";
import JoystickPad from "../panels/JoystickPad";

interface RobotPageLayoutProps {
  client: FoxgloveClientHandle;
  accordionItems: AccordionItem[];
  defaultOpen?: string[];
  viewerMode?: ViewerMode;
  extraPanels?: ReactNode;
  extraOverlay?: ReactNode;
}

export default function RobotPageLayout({
  client,
  accordionItems,
  defaultOpen = [],
  viewerMode,
  extraPanels,
  extraOverlay,
}: RobotPageLayoutProps) {
  const { enabled: vizEnabled, layers, overlays } = useVisualization();
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
                  <CameraImagePanel client={client} className="w-full h-full" />
                </div>
              )}
              {layers.depthImage && (
                <div className="flex-1 rounded-lg overflow-hidden border border-gray-700">
                  <DepthImagePanel client={client} className="w-full h-full" />
                </div>
              )}
            </div>
          )}
          {extraPanels}
        </div>
      )}
    </div>
  );
}
