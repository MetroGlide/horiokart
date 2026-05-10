export type LabeledValueVariant =
  | "normal"
  | "success"
  | "warn"
  | "error"
  | "muted";

const VALUE_CLASS: Record<LabeledValueVariant, string> = {
  normal: "text-white",
  success: "text-green-400",
  warn: "text-yellow-400",
  error: "text-red-400",
  muted: "text-gray-400",
};

interface LabeledValueProps {
  label: string;
  value: string;
  variant?: LabeledValueVariant;
  bold?: boolean;
}

export default function LabeledValue({
  label,
  value,
  variant = "normal",
  bold = false,
}: LabeledValueProps) {
  return (
    <div>
      <span className="block text-xs text-gray-400">{label}</span>
      <p className={`${VALUE_CLASS[variant]} ${bold ? "font-semibold" : ""}`}>
        {value}
      </p>
    </div>
  );
}
