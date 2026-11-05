/** One second record of /api/measurements, see src/dashboard/history.py */
export interface Measurement {
  ts: number;
  true_wind_speed_mps: number;
  true_wind_dir_deg: number;
  apparent_wind_speed_mps: number;
  apparent_wind_dir_deg: number;
  sensor_speed_mps?: number | null;
  sensor_heading_deg?: number | null;
  gps_lat?: number | null;
  gps_lng?: number | null;
}

export interface MeasurementsResponse {
  server_time: number;
  items: Measurement[];
}

/** ms until a request counts as failed, so a stalled connection isn't waited on */
export const REQUEST_TIMEOUT_MS = 5000;

/** Records newer than `since` (ms), only the newest `limit` of them if given */
export async function fetchMeasurements(
  since?: number,
  limit?: number,
  timeoutMs = REQUEST_TIMEOUT_MS,
): Promise<MeasurementsResponse> {
  const query = new URLSearchParams({ since: since?.toString() ?? "" });
  if (limit !== undefined) query.set("limit", limit.toString());
  const response = await fetch(`/api/measurements?${query}`, {
    signal: AbortSignal.timeout(timeoutMs),
  });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return (await response.json()) as MeasurementsResponse;
}

/** A section of src/telemetry.py: live values of one part of the pipeline */
export type TelemetrySection = Record<string, unknown> & {
  /** ms since epoch */
  updated?: number;
};

export interface StatusResponse {
  /** ms since epoch */
  server_time: number;
  values: Record<string, TelemetrySection>;
  mqtt: {
    enabled: boolean;
    broker?: string;
    topic?: string;
    state?: string | null;
    error_count?: number;
  };
  system: {
    uptime: number | null;
    load: number | null;
    cpu_temperature: number | null;
    memory_total_mb: number | null;
    memory_available_mb: number | null;
    interfaces: { name: string; up: boolean; addresses: string[] }[];
    throttled: string | null;
    /** null until the values were read once */
    disk_free_mb: number | null;
    disk_total_mb: number | null;
  };
}

export async function fetchStatus(): Promise<StatusResponse> {
  const response = await fetch("/api/status", {
    signal: AbortSignal.timeout(REQUEST_TIMEOUT_MS),
  });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return (await response.json()) as StatusResponse;
}

export interface KnownNetwork {
  uuid: string;
  ssid: string;
  autoconnect: boolean;
  priority: number;
  active: boolean;
}

export interface WifiStatus {
  mode: "client" | "searching" | "hotspot" | "unavailable";
  ssid: string | null;
  address: string | null;
  signal: number | null;
  /** devices connected to the hotspot */
  clients: number | null;
  /** seconds until the hotspot opens unless a known network connects */
  hotspot_in: number | null;
  hotspot_enabled: boolean;
  hotspot_ssid: string;
  hotspot_address: string;
  /** what the windbot is doing right now, e.g. connecting to a network */
  action: string | null;
  message: string | null;
  error: string | null;
  networks: KnownNetwork[];
}

export interface ScannedNetwork {
  ssid: string;
  signal: number | null;
  security: string;
  in_use: boolean;
}

/** ms a scan may take: the windbot rescans for up to 30 s, then lists what it knows */
export const SCAN_TIMEOUT_MS = 45_000;

async function getJson<T>(path: string, timeoutMs = REQUEST_TIMEOUT_MS): Promise<T> {
  const response = await fetch(path, { signal: AbortSignal.timeout(timeoutMs) });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(data.error ?? `Request failed (${response.status})`);
  return data as T;
}

const PASSWORD_KEY = "windbotAdminPassword";
/**
 * ms a change may take. Connecting runs in the background on the windbot, but some
 * changes wait for nmcli, which is slow on a Pi Zero.
 */
const POST_TIMEOUT_MS = 15_000;

/**
 * POST a change. If the windbot asks for the admin password (ADMIN_PASSWORD in
 * config.ini), it is asked for once and kept for this browser tab.
 */
export async function postJson<T = unknown>(path: string, body: unknown): Promise<T> {
  for (let attempt = 0; attempt < 2; attempt++) {
    const response = await fetch(path, {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        "X-Admin-Password": sessionStorage.getItem(PASSWORD_KEY) ?? "",
      },
      body: JSON.stringify(body),
      signal: AbortSignal.timeout(POST_TIMEOUT_MS),
    });
    const data = await response.json().catch(() => ({}));
    if (response.status === 401 && attempt === 0) {
      const password = prompt("Admin password of the windbot");
      if (password == null) throw new Error("An admin password is needed for this.");
      sessionStorage.setItem(PASSWORD_KEY, password);
      continue;
    }
    if (!response.ok) throw new Error(data.error ?? `Request failed (${response.status})`);
    return data as T;
  }
  throw new Error("Wrong admin password.");
}

export const fetchWifi = () => getJson<WifiStatus>("/api/wifi");
export const scanWifi = () =>
  getJson<{ networks: ScannedNetwork[] }>("/api/wifi/scan", SCAN_TIMEOUT_MS);
