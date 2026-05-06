import SectionCard from "../layout/SectionCard";
import StatusBadge from "../ui/StatusBadge";

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
      <StatusBadge status={status} />
    </SectionCard>
  );
}
