interface SectionLabelProps {
  text: string;
}

export default function SectionLabel({ text }: SectionLabelProps) {
  return <p className="text-xs text-gray-400">{text}</p>;
}
