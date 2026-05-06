import { ReactNode } from "react";

interface SectionCardProps {
  title?: string;
  children: ReactNode;
  className?: string;
}

export default function SectionCard({
  title,
  children,
  className = "",
}: SectionCardProps) {
  return (
    <div
      className={`bg-gray-900 border border-gray-600/60 rounded-md p-3 ${className}`}
    >
      {title && (
        <p className="text-xs font-medium text-gray-400 mb-2">{title}</p>
      )}
      {children}
    </div>
  );
}
