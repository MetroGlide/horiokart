import {
  createContext,
  useContext,
  useState,
  useEffect,
  ReactNode,
} from "react";
import { loadSettings, saveSettings } from "../utils/settingsApi";

interface SimulationContextType {
  isSimulation: boolean;
  setIsSimulation: (val: boolean) => void;
}

const SimulationContext = createContext<SimulationContextType>({
  isSimulation: false,
  setIsSimulation: () => {},
});

export function SimulationProvider({ children }: { children: ReactNode }) {
  const [isSimulation, setIsSimulationState] = useState<boolean>(false);

  useEffect(() => {
    loadSettings().then((data) => {
      const s = data.simulation as { isSimulation?: boolean } | undefined;
      if (s?.isSimulation !== undefined)
        setIsSimulationState(Boolean(s.isSimulation));
    });
  }, []);

  const setIsSimulation = (val: boolean) => {
    setIsSimulationState(val);
    saveSettings("simulation", { isSimulation: val });
  };

  return (
    <SimulationContext.Provider value={{ isSimulation, setIsSimulation }}>
      {children}
    </SimulationContext.Provider>
  );
}

export function useSimulation() {
  return useContext(SimulationContext);
}
