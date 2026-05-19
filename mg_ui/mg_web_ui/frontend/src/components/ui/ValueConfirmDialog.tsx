interface ValueConfirmDialogProps {
  open: boolean;
  title: string;
  values: { label: string; value: string }[];
  onConfirm: () => void;
  onCancel: () => void;
}

export default function ValueConfirmDialog({
  open,
  title,
  values,
  onConfirm,
  onCancel,
}: ValueConfirmDialogProps) {
  if (!open) return null;

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center">
      <div
        className="absolute inset-0 bg-black/60 backdrop-blur-sm"
        onClick={onCancel}
      />
      <div className="relative z-10 bg-gray-900 rounded-xl shadow-2xl w-full max-w-lg">
        <div className="px-5 py-4 border-b border-gray-700">
          <span className="text-sm font-medium text-gray-200">{title}</span>
        </div>
        <div className="px-5 py-4 space-y-3">
          {values.map(({ label, value }) => (
            <div key={label}>
              <p className="text-xs text-gray-400 mb-1">{label}</p>
              <p className="text-sm text-white bg-gray-800 rounded px-3 py-2 break-all">
                {value || <span className="text-gray-500 italic">（空）</span>}
              </p>
            </div>
          ))}
        </div>
        <div className="flex justify-end gap-2 px-5 py-3 border-t border-gray-700">
          <button
            onClick={onCancel}
            className="px-4 py-2 rounded text-sm font-medium bg-gray-700 hover:bg-gray-600 text-gray-200"
          >
            Cancel
          </button>
          <button
            onClick={onConfirm}
            className="px-4 py-2 rounded text-sm font-medium bg-blue-600 hover:bg-blue-700 text-white"
          >
            OK
          </button>
        </div>
      </div>
    </div>
  );
}
