import { Routes, Route } from "react-router-dom";
import { useFoxgloveClient } from "./hooks/useFoxgloveClient";
import TopPage from "./pages/TopPage";
import WaypointNavPage from "./pages/WaypointNavPage";
import SlamPage from "./pages/SlamPage";
import UtilityPage from "./pages/UtilityPage";
import SimulationPage from "./pages/SimulationPage";
import NavBar from "./components/NavBar";
import ConnectionBadge from "./components/ConnectionBadge";

export default function App() {
  const client = useFoxgloveClient();

  return (
    <div className="min-h-screen bg-gray-900 text-white">
      <header className="flex items-center justify-between px-4 py-3 bg-gray-800 shadow-md">
        <span className="text-lg font-bold tracking-wide">
          MG-01 Control UI
        </span>
        <ConnectionBadge status={client.status} />
      </header>
      <NavBar />
      <main className="p-4 max-w-4xl mx-auto">
        <Routes>
          <Route path="/" element={<TopPage client={client} />} />
          <Route
            path="/waypoint"
            element={<WaypointNavPage client={client} />}
          />
          <Route path="/slam" element={<SlamPage client={client} />} />
          <Route
            path="/simulation"
            element={<SimulationPage client={client} />}
          />
          <Route path="/utility" element={<UtilityPage client={client} />} />
        </Routes>
      </main>
    </div>
  );
}
