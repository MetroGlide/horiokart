import SectionCard from "../layout/SectionCard";

export interface ServiceButton {
  label: string;
  onClick: () => void;
  variant?: "green" | "red" | "blue" | "gray";
}

interface ServiceControlCardProps {
  title: string;
  buttons: ServiceButton[];
  loading?: boolean;
  error?: string | null;
}

const VARIANT_CLASS: Record<NonNullable<ServiceButton["variant"]>, string> = {
  green: "bg-green-600 hover:bg-green-700",
  red: "bg-red-600 hover:bg-red-700",
  blue: "bg-blue-600 hover:bg-blue-700",
  gray: "bg-gray-600 hover:bg-gray-500",
};

export default function ServiceControlCard({
  title,
  buttons,
  loading = false,
  error,
}: ServiceControlCardProps) {
  return (
    <SectionCard title={title}>
      <div className="flex flex-wrap gap-3">
        {buttons.map((btn) => (
          <button
            key={btn.label}
            onClick={btn.onClick}
            disabled={loading}
            className={`${VARIANT_CLASS[btn.variant ?? "gray"]} disabled:opacity-50 px-4 py-2 rounded font-medium text-sm`}
          >
            {btn.label}
          </button>
        ))}
      </div>
      {error && <p className="text-red-400 text-sm mt-2">{error}</p>}
    </SectionCard>
  );
}
