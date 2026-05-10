import { useNavigate } from "react-router-dom";
import SettingPage from "../pages/SettingPage";

interface SettingModalProps {
  open: boolean;
  onClose: () => void;
}

export default function SettingModal({ open, onClose }: SettingModalProps) {
  const navigate = useNavigate();

  if (!open) return null;

  const handleExpand = () => {
    onClose();
    navigate("/setting");
  };

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center"
      onMouseDown={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
    >
      <div
        className="absolute inset-0 bg-black/60 backdrop-blur-sm"
        onClick={onClose}
      />
      <div className="relative z-10 bg-gray-900 rounded-xl shadow-2xl w-full max-w-2xl max-h-[80vh] flex flex-col">
        <div className="flex items-center justify-between px-4 py-3 border-b border-gray-700 flex-shrink-0">
          <span className="text-sm font-medium text-gray-300">Settings</span>
          <div className="flex items-center gap-1">
            <button
              onClick={handleExpand}
              className="p-1.5 text-gray-400 hover:text-white rounded transition-colors"
              title="フルページで開く"
            >
              <svg
                className="w-4 h-4"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M4 8V4m0 0h4M4 4l5 5m11-1V4m0 0h-4m4 0l-5 5M4 16v4m0 0h4m-4 0l5-5m11 5l-5-5m5 5v-4m0 4h-4"
                />
              </svg>
            </button>
            <button
              onClick={onClose}
              className="p-1.5 text-gray-400 hover:text-white rounded transition-colors"
              title="閉じる"
            >
              <svg
                className="w-4 h-4"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M6 18L18 6M6 6l12 12"
                />
              </svg>
            </button>
          </div>
        </div>
        <div className="overflow-y-auto p-4">
          <SettingPage />
        </div>
      </div>
    </div>
  );
}
