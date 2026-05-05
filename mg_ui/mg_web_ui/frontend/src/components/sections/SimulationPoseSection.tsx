import { useState } from "react";
import SectionCard from "../SectionCard";

export interface PoseInput {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

interface SimulationPoseSectionProps {
  onResetRobot: (pose: PoseInput) => void;
  onResetAmcl?: (pose: PoseInput) => void;
  loading?: boolean;
  error?: string | null;
}

export default function SimulationPoseSection({
  onResetRobot,
  onResetAmcl,
  loading = false,
  error,
}: SimulationPoseSectionProps) {
  const [pose, setPose] = useState<PoseInput>({
    x: 0.0,
    y: 0.0,
    z: 0.05,
    yaw: 0.0,
  });

  const update = (key: keyof PoseInput, value: number) =>
    setPose((prev) => ({ ...prev, [key]: value }));

  return (
    <SectionCard title="ポーズリセット">
      <div className="grid grid-cols-2 gap-3 mb-3">
        {(["x", "y", "z", "yaw"] as const).map((k) => (
          <label key={k} className="text-sm text-gray-300">
            <span className="block text-xs text-gray-400 mb-1">
              {k === "yaw" ? "yaw(rad)" : k}
            </span>
            <input
              type="number"
              step={k === "z" ? "0.01" : "0.1"}
              value={pose[k]}
              onChange={(e) => update(k, Number(e.target.value))}
              className="w-full bg-gray-700 rounded px-2 py-1 text-sm"
            />
          </label>
        ))}
      </div>
      <div className="flex flex-wrap gap-2">
        <button
          onClick={() => onResetRobot(pose)}
          disabled={loading}
          className="bg-blue-600 hover:bg-blue-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
        >
          Reset Robot Pose
        </button>
        {onResetAmcl && (
          <button
            onClick={() => onResetAmcl(pose)}
            disabled={loading}
            className="bg-indigo-600 hover:bg-indigo-700 disabled:opacity-50 px-4 py-2 rounded font-medium text-sm"
          >
            Reset AMCL Pose
          </button>
        )}
      </div>
      {error && <p className="text-red-400 text-sm mt-2">{error}</p>}
    </SectionCard>
  );
}
