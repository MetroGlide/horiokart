const STORAGE_KEY = "sysManagerBaseUrl";

function defaultUrl(): string {
  return `http://${window.location.hostname}:8001`;
}

export function getSysManagerUrl(): string {
  return localStorage.getItem(STORAGE_KEY) || defaultUrl();
}

export function getSysManagerDefaultUrl(): string {
  return defaultUrl();
}

export function setSysManagerUrl(url: string): void {
  if (url.trim()) {
    localStorage.setItem(STORAGE_KEY, url.trim());
  } else {
    localStorage.removeItem(STORAGE_KEY);
  }
}

export function resetSysManagerUrl(): void {
  localStorage.removeItem(STORAGE_KEY);
}
