import { useState } from "react";
import { Routes, Route } from "react-router-dom";
import { useFoxgloveClient } from "./hooks/useFoxgloveClient";
import { useSystemManagerClient } from "./hooks/useSystemManagerClient";
import { SimulationProvider } from "./contexts/SimulationContext";
import { VisualizationProvider } from "./contexts/VisualizationContext";
import { TeleopProvider } from "./contexts/TeleopContext";
import TopPage from "./pages/TopPage";
import WaypointNavPage from "./pages/WaypointNavPage";
import SlamPage from "./pages/SlamPage";
import SystemPage from "./pages/SystemPage";
import SettingPage from "./pages/SettingPage";
import NavBar from "./components/NavBar";
import ConnectionBadge from "./components/ConnectionBadge";
import SettingModal from "./components/SettingModal";

export default function App() {
  const client = useFoxgloveClient();
  const sysManager = useSystemManagerClient();
  const [settingOpen, setSettingOpen] = useState(false);

  return (
    <SimulationProvider>
      <VisualizationProvider>
        <TeleopProvider>
          <div className="min-h-screen bg-gray-900 text-white">
            <header className="flex items-center justify-between px-4 py-3 bg-gray-800 shadow-md">
              <span className="text-lg font-bold tracking-wide">
                MG-01 Control UI
              </span>
              <ConnectionBadge status={client.status} />
            </header>
            <NavBar onSettingClick={() => setSettingOpen(true)} />
            <main className="p-4">
              <Routes>
                <Route path="/" element={<TopPage client={client} />} />
                <Route
                  path="/waypoint"
                  element={
                    <WaypointNavPage client={client} sysManager={sysManager} />
                  }
                />
                <Route
                  path="/slam"
                  element={<SlamPage client={client} sysManager={sysManager} />}
                />
                <Route
                  path="/system"
                  element={
                    <SystemPage client={client} sysManager={sysManager} />
                  }
                />
                <Route path="/setting" element={<SettingPage />} />
              </Routes>
            </main>
            <SettingModal
              open={settingOpen}
              onClose={() => setSettingOpen(false)}
            />
          </div>
        </TeleopProvider>
      </VisualizationProvider>
    </SimulationProvider>
  );
}
