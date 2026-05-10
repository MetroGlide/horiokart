interface ToggleProps {
  value: boolean;
  onChange: () => void;
  disabled?: boolean;
}

export default function Toggle({
  value,
  onChange,
  disabled = false,
}: ToggleProps) {
  return (
    <button
      onClick={onChange}
      disabled={disabled}
      className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors disabled:opacity-50 ${
        value ? "bg-blue-600" : "bg-gray-600"
      }`}
    >
      <span
        className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
          value ? "translate-x-6" : "translate-x-1"
        }`}
      />
    </button>
  );
}
