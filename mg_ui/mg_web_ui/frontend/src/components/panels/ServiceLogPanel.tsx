import { useEffect, useMemo, useRef, useState } from "react";
import type { LogEntry } from "../../types/api";

function escapeRegex(value: string): string {
  return value.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
}

function HighlightText({ text, keyword }: { text: string; keyword: string }) {
  if (!keyword) return <>{text}</>;
  const parts = text.split(new RegExp(`(${escapeRegex(keyword)})`, "gi"));
  return (
    <>
      {parts.map((part, index) =>
        part.toLowerCase() === keyword.toLowerCase() ? (
          <mark
            key={`${part}-${index}`}
            className="bg-yellow-300 text-black px-0.5 rounded-sm"
          >
            {part}
          </mark>
        ) : (
          <span key={`${part}-${index}`}>{part}</span>
        ),
      )}
    </>
  );
}

export default function ServiceLogPanel({
  entries,
  displayServices,
  connected,
  receivingServices,
  onClear,
}: {
  entries: LogEntry[];
  displayServices: Set<string>;
  connected: boolean;
  receivingServices: Set<string>;
  onClear: () => void;
}) {
  const [filterText, setFilterText] = useState("");
  const [autoScroll, setAutoScroll] = useState(true);
  const scrollRef = useRef<HTMLDivElement | null>(null);

  const visibleEntries = useMemo(() => {
    const keyword = filterText.trim().toLowerCase();
    return entries.filter((entry) => {
      if (!displayServices.has(entry.service)) return false;
      if (!keyword) return true;
      return entry.line.toLowerCase().includes(keyword);
    });
  }, [entries, displayServices, filterText]);

  const displayOnlyServices = useMemo(() => {
    return [...displayServices].filter(
      (service) => !receivingServices.has(service),
    );
  }, [displayServices, receivingServices]);

  useEffect(() => {
    if (!autoScroll || !scrollRef.current) return;
    const node = scrollRef.current;
    node.scrollTop = node.scrollHeight;
  }, [visibleEntries, autoScroll]);

  if (entries.length === 0 && displayOnlyServices.length === 0) {
    return null;
  }

  return (
    <section className="bg-gray-800 rounded-lg p-4 space-y-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-3">
          <p className="text-xs text-gray-400">Service Log</p>
          <span
            className={`text-xs font-semibold ${connected ? "text-green-400" : "text-yellow-400"}`}
          >
            {connected ? "Connected" : "Reconnecting"}
          </span>
        </div>
        <button
          type="button"
          onClick={onClear}
          className="text-xs text-gray-300 hover:text-white"
        >
          Clear
        </button>
      </div>

      <div className="flex items-center gap-3">
        <input
          value={filterText}
          onChange={(e) => setFilterText(e.target.value)}
          placeholder="Filter keyword"
          className="flex-1 min-w-0 bg-gray-700 text-gray-100 text-sm rounded px-2 py-1"
        />
        <label className="flex items-center gap-1 text-xs text-gray-300 select-none">
          <input
            type="checkbox"
            checked={autoScroll}
            onChange={(e) => setAutoScroll(e.target.checked)}
          />
          Auto Scroll
        </label>
      </div>

      {displayOnlyServices.length > 0 && (
        <p className="text-xs text-yellow-300">
          表示対象で受信OFFのサービス: {displayOnlyServices.join(", ")}
        </p>
      )}

      <div
        ref={scrollRef}
        className="max-h-96 overflow-y-auto rounded bg-gray-900 p-2 font-mono text-xs space-y-1"
      >
        {visibleEntries.length === 0 ? (
          <p className="text-gray-500">No logs for selected services.</p>
        ) : (
          visibleEntries.map((entry) => (
            <div key={entry.id} className="flex gap-2 items-start">
              <span className="shrink-0 text-cyan-300">[{entry.service}]</span>
              <span className="text-gray-200 break-all">
                <HighlightText text={entry.line} keyword={filterText.trim()} />
              </span>
            </div>
          ))
        )}
      </div>
    </section>
  );
}
