// Formatting shared by the views. The wind speed unit is the one chosen in the settings
// of the wind charts.

export type WindSpeedUnit = "mps" | "knots" | "kmh";

const FACTOR: Record<WindSpeedUnit, number> = { mps: 1, knots: 1.943844, kmh: 3.6 };
const LABEL: Record<WindSpeedUnit, string> = { mps: "m/s", knots: "kn", kmh: "km/h" };

export function storedWindSpeedUnit(): WindSpeedUnit {
  try {
    const unit = JSON.parse(localStorage.getItem("windDashboardSettings") ?? "{}").windSpeedUnit;
    return unit in FACTOR ? unit : "knots";
  } catch {
    return "knots";
  }
}

const isNumber = (value: unknown): value is number =>
  typeof value === "number" && Number.isFinite(value);

export function formatSpeed(mps: unknown, unit: WindSpeedUnit): string {
  return isNumber(mps) ? `${(mps * FACTOR[unit]).toFixed(1)} ${LABEL[unit]}` : "—";
}

export function formatDirection(deg: unknown): string {
  return isNumber(deg) ? `${Math.floor(((deg % 360) + 360) % 360)}°` : "—";
}

export function formatNumber(value: unknown, digits = 1, suffix = ""): string {
  return isNumber(value) ? `${value.toFixed(digits)}${suffix}` : "—";
}

/** e.g. 45s, 3m05s, 2h10m */
export function formatAge(sec: number): string {
  if (sec >= 3600)
    return `${Math.floor(sec / 3600)}h${String(Math.floor((sec % 3600) / 60)).padStart(2, "0")}m`;
  if (sec >= 60) return `${Math.floor(sec / 60)}m${String(sec % 60).padStart(2, "0")}s`;
  return `${sec}s`;
}
