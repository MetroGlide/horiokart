import SectionCard from "../SectionCard";

interface ContainerStatusCardProps {
  title: string;
  status: string;
}

export default function ContainerStatusCard({
  title,
  status,
}: ContainerStatusCardProps) {
  return (
    <SectionCard title={title}>
      <div className="flex items-center gap-3">
        <span
          className={`w-3 h-3 rounded-full ${status === "running" ? "bg-green-500" : "bg-gray-500"}`}
        />
        <span className="text-lg font-semibold capitalize">{status}</span>
      </div>
    </SectionCard>
  );
}
