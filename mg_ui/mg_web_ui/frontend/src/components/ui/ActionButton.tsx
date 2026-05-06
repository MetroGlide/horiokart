export type ButtonVariant = "green" | "red" | "yellow" | "blue" | "gray";
export type ButtonSize = "sm" | "md";

const VARIANT_CLASS: Record<ButtonVariant, string> = {
  green: "bg-green-600 hover:bg-green-700",
  red: "bg-red-600 hover:bg-red-700",
  yellow: "bg-yellow-600 hover:bg-yellow-700",
  blue: "bg-blue-600 hover:bg-blue-700",
  gray: "bg-gray-600 hover:bg-gray-500",
};

const SIZE_CLASS: Record<ButtonSize, string> = {
  sm: "px-3 py-1 text-xs",
  md: "px-4 py-2 text-sm",
};

interface ActionButtonProps {
  label: string;
  onClick: () => void;
  variant?: ButtonVariant;
  size?: ButtonSize;
  disabled?: boolean;
  loading?: boolean;
}

export default function ActionButton({
  label,
  onClick,
  variant = "gray",
  size = "md",
  disabled = false,
  loading = false,
}: ActionButtonProps) {
  return (
    <button
      onClick={onClick}
      disabled={disabled || loading}
      className={`${VARIANT_CLASS[variant]} ${SIZE_CLASS[size]} disabled:opacity-50 rounded font-medium`}
    >
      {label}
    </button>
  );
}
