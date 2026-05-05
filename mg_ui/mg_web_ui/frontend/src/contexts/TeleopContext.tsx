import {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  useRef,
  ReactNode,
} from "react";
import { loadSettings, saveSettings } from "../utils/settingsApi";

interface TeleopSettings {
  maxLinear: number;
  maxAngular: number;
  gaugeMaxLinear: number;
  gaugeMaxAngular: number;
  gaugeSyncWithPad: boolean;
}

interface TeleopContextType extends TeleopSettings {
  setMaxLinear: (val: number) => void;
  setMaxAngular: (val: number) => void;
  setGaugeMaxLinear: (val: number) => void;
  setGaugeMaxAngular: (val: number) => void;
  setGaugeSyncWithPad: (val: boolean) => void;
  effectiveGaugeMaxLinear: number;
  effectiveGaugeMaxAngular: number;
}

const DEFAULT_TELEOP: TeleopSettings = {
  maxLinear: 0.5,
  maxAngular: 0.5,
  gaugeMaxLinear: 1.0,
  gaugeMaxAngular: 1.0,
  gaugeSyncWithPad: true,
};

const TeleopContext = createContext<TeleopContextType>({
  ...DEFAULT_TELEOP,
  setMaxLinear: () => {},
  setMaxAngular: () => {},
  setGaugeMaxLinear: () => {},
  setGaugeMaxAngular: () => {},
  setGaugeSyncWithPad: () => {},
  effectiveGaugeMaxLinear: DEFAULT_TELEOP.maxLinear,
  effectiveGaugeMaxAngular: DEFAULT_TELEOP.maxAngular,
});

export function TeleopProvider({ children }: { children: ReactNode }) {
  const [maxLinear, setMaxLinearState] = useState(DEFAULT_TELEOP.maxLinear);
  const [maxAngular, setMaxAngularState] = useState(DEFAULT_TELEOP.maxAngular);
  const [gaugeMaxLinear, setGaugeMaxLinearState] = useState(
    DEFAULT_TELEOP.gaugeMaxLinear,
  );
  const [gaugeMaxAngular, setGaugeMaxAngularState] = useState(
    DEFAULT_TELEOP.gaugeMaxAngular,
  );
  const [gaugeSyncWithPad, setGaugeSyncWithPadState] = useState(
    DEFAULT_TELEOP.gaugeSyncWithPad,
  );
  const saveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  useEffect(() => {
    loadSettings().then((data) => {
      const t = data.teleop as Partial<TeleopSettings> | undefined;
      if (!t) return;
      if (typeof t.maxLinear === "number") setMaxLinearState(t.maxLinear);
      if (typeof t.maxAngular === "number") setMaxAngularState(t.maxAngular);
      if (typeof t.gaugeMaxLinear === "number")
        setGaugeMaxLinearState(t.gaugeMaxLinear);
      if (typeof t.gaugeMaxAngular === "number")
        setGaugeMaxAngularState(t.gaugeMaxAngular);
      if (typeof t.gaugeSyncWithPad === "boolean")
        setGaugeSyncWithPadState(t.gaugeSyncWithPad);
    });
  }, []);

  const persist = useCallback(
    (lin: number, ang: number, gLin: number, gAng: number, sync: boolean) => {
      if (saveTimerRef.current) clearTimeout(saveTimerRef.current);
      saveTimerRef.current = setTimeout(() => {
        saveSettings("teleop", {
          maxLinear: lin,
          maxAngular: ang,
          gaugeMaxLinear: gLin,
          gaugeMaxAngular: gAng,
          gaugeSyncWithPad: sync,
        });
      }, 500);
    },
    [],
  );

  const setMaxLinear = (val: number) => {
    setMaxLinearState(val);
    persist(val, maxAngular, gaugeMaxLinear, gaugeMaxAngular, gaugeSyncWithPad);
  };

  const setMaxAngular = (val: number) => {
    setMaxAngularState(val);
    persist(maxLinear, val, gaugeMaxLinear, gaugeMaxAngular, gaugeSyncWithPad);
  };

  const setGaugeMaxLinear = (val: number) => {
    setGaugeMaxLinearState(val);
    persist(maxLinear, maxAngular, val, gaugeMaxAngular, gaugeSyncWithPad);
  };

  const setGaugeMaxAngular = (val: number) => {
    setGaugeMaxAngularState(val);
    persist(maxLinear, maxAngular, gaugeMaxLinear, val, gaugeSyncWithPad);
  };

  const setGaugeSyncWithPad = (val: boolean) => {
    setGaugeSyncWithPadState(val);
    persist(maxLinear, maxAngular, gaugeMaxLinear, gaugeMaxAngular, val);
  };

  const effectiveGaugeMaxLinear = gaugeSyncWithPad ? maxLinear : gaugeMaxLinear;
  const effectiveGaugeMaxAngular = gaugeSyncWithPad
    ? maxAngular
    : gaugeMaxAngular;

  return (
    <TeleopContext.Provider
      value={{
        maxLinear,
        maxAngular,
        gaugeMaxLinear,
        gaugeMaxAngular,
        gaugeSyncWithPad,
        setMaxLinear,
        setMaxAngular,
        setGaugeMaxLinear,
        setGaugeMaxAngular,
        setGaugeSyncWithPad,
        effectiveGaugeMaxLinear,
        effectiveGaugeMaxAngular,
      }}
    >
      {children}
    </TeleopContext.Provider>
  );
}

export function useTeleop() {
  return useContext(TeleopContext);
}
