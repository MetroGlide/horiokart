import { useSimulation } from "../contexts/SimulationContext"

export default function SettingPage() {
  const { isSimulation, setIsSimulation } = useSimulation()

  return (
    <div className="space-y-6">
      <section className="bg-gray-800 rounded-lg p-4 space-y-4">
        <p className="text-xs text-gray-400">Simulation</p>
        <div className="flex items-center gap-4">
          <span className="text-sm text-gray-300">Simulation Mode</span>
          <button
            onClick={() => setIsSimulation(!isSimulation)}
            className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors ${
              isSimulation ? "bg-blue-600" : "bg-gray-600"
            }`}
          >
            <span
              className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
                isSimulation ? "translate-x-6" : "translate-x-1"
              }`}
            />
          </button>
          <span className={`text-sm font-semibold ${isSimulation ? "text-blue-400" : "text-gray-500"}`}>
            {isSimulation ? "ON" : "OFF"}
          </span>
        </div>
        <p className="text-xs text-gray-500">
          ON にすると各ページの Simulation 関連機能が表示されます。設定は保持されます。
        </p>
      </section>

      <section className="bg-gray-800 rounded-lg p-4 space-y-2">
        <p className="text-xs text-gray-400">Display Settings</p>
        <p className="text-sm text-gray-500">（将来的に各ページの表示機能切り替えを追加予定）</p>
      </section>
    </div>
  )
}
