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
