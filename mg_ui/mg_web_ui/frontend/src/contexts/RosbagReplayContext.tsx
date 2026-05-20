import {
  createContext,
  useContext,
  useState,
  useEffect,
  ReactNode,
} from "react";
import { loadSettings, saveSettings } from "../utils/settingsApi";

interface RosbagReplayContextType {
  isRosbagReplayVisible: boolean;
  setIsRosbagReplayVisible: (val: boolean) => void;
}

const RosbagReplayContext = createContext<RosbagReplayContextType>({
  isRosbagReplayVisible: false,
  setIsRosbagReplayVisible: () => {},
});

export function RosbagReplayProvider({ children }: { children: ReactNode }) {
  const [isRosbagReplayVisible, setIsRosbagReplayVisibleState] =
    useState<boolean>(false);

  useEffect(() => {
    loadSettings().then((data) => {
      const s = data.rosbagReplay as
        | { isRosbagReplayVisible?: boolean }
        | undefined;
      if (s?.isRosbagReplayVisible !== undefined)
        setIsRosbagReplayVisibleState(Boolean(s.isRosbagReplayVisible));
    });
  }, []);

  const setIsRosbagReplayVisible = (val: boolean) => {
    setIsRosbagReplayVisibleState(val);
    saveSettings("rosbagReplay", { isRosbagReplayVisible: val });
  };

  return (
    <RosbagReplayContext.Provider
      value={{ isRosbagReplayVisible, setIsRosbagReplayVisible }}
    >
      {children}
    </RosbagReplayContext.Provider>
  );
}

export function useRosbagReplay() {
  return useContext(RosbagReplayContext);
}
