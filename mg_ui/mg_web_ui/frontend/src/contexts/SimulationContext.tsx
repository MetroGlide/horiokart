import { createContext, useContext, useState, ReactNode } from "react"

interface SimulationContextType {
  isSimulation: boolean
  setIsSimulation: (val: boolean) => void
}

const SimulationContext = createContext<SimulationContextType>({
  isSimulation: false,
  setIsSimulation: () => {},
})

export function SimulationProvider({ children }: { children: ReactNode }) {
  const [isSimulation, setIsSimulationState] = useState<boolean>(() => {
    return localStorage.getItem("simulationMode") === "true"
  })

  const setIsSimulation = (val: boolean) => {
    localStorage.setItem("simulationMode", String(val))
    setIsSimulationState(val)
  }

  return (
    <SimulationContext.Provider value={{ isSimulation, setIsSimulation }}>
      {children}
    </SimulationContext.Provider>
  )
}

export function useSimulation() {
  return useContext(SimulationContext)
}
