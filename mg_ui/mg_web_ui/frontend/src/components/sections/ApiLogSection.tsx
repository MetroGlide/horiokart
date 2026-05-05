import { ApiLog } from "../../hooks/useSystemManagerClient";
import ApiLogPanel from "../ApiLogPanel";

interface ApiLogSectionProps {
  logs: ApiLog[];
}

export default function ApiLogSection({ logs }: ApiLogSectionProps) {
  return <ApiLogPanel logs={logs} />;
}
