import { getSysManagerUrl } from "./systemManagerConfig";

let _cache: Record<string, unknown> = {};

export async function loadSettings(): Promise<Record<string, unknown>> {
  try {
    const data = await fetch(`${getSysManagerUrl()}/settings`).then((r) => r.json());
    _cache = typeof data === "object" && data !== null ? (data as Record<string, unknown>) : {};
    return _cache;
  } catch {
    return {};
  }
}

export function saveSettings(key: string, value: unknown): void {
  _cache = { ..._cache, [key]: value };
  fetch(`${getSysManagerUrl()}/settings`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(_cache),
  }).catch(() => {});
}
