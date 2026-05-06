import { useState, ReactNode } from "react";

export interface AccordionItem {
  id: string;
  label: string;
  children: ReactNode;
}

interface SideAccordionProps {
  items: AccordionItem[];
  defaultOpen?: string[];
}

export default function SideAccordion({
  items,
  defaultOpen = [],
}: SideAccordionProps) {
  const [openIds, setOpenIds] = useState<Set<string>>(new Set(defaultOpen));

  const toggle = (id: string) => {
    setOpenIds((prev) => {
      const next = new Set(prev);
      if (next.has(id)) {
        next.delete(id);
      } else {
        next.add(id);
      }
      return next;
    });
  };

  return (
    <div className="space-y-1">
      {items.map(({ id, label, children }) => (
        <div key={id} className="bg-gray-800 rounded-lg overflow-hidden">
          <button
            onClick={() => toggle(id)}
            className="w-full flex items-center justify-between px-4 py-2.5 text-sm font-medium text-gray-300 hover:text-white hover:bg-gray-700 transition-colors"
          >
            <span>{label}</span>
            <svg
              className={`w-4 h-4 flex-shrink-0 transform transition-transform ${openIds.has(id) ? "rotate-90" : ""}`}
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M9 5l7 7-7 7"
              />
            </svg>
          </button>
          {openIds.has(id) && <div className="px-4 pb-4 pt-1">{children}</div>}
        </div>
      ))}
    </div>
  );
}
