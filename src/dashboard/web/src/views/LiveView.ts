// Live values of the pipeline and the system, to see why wind might not be showing up
// without SSH and logs.
import { computed, defineComponent, onMounted, onUnmounted, ref } from "vue";
import { fetchStatus, type StatusResponse, type TelemetrySection } from "../api";
import {
  formatAge,
  formatDirection,
  formatNumber,
  formatSpeed,
  storedWindSpeedUnit,
} from "../format";

export default defineComponent({
  setup() {
    interface Card {
      title: string;
      /** null: the section was never published, undefined: the card has no age */
      section?: TelemetrySection | null;
      rows: [string, string | number][];
    }

    /** ms between the end of one request and the start of the next */
    const POLL_INTERVAL_MS = 1000;

    const status = ref<StatusResponse | null>(null);
    const unreachable = ref(false);
    const unit = storedWindSpeedUnit();
    let timer: number | undefined;
    let stopped = false;

    // One request at a time, like on the Wind tab. A stalled request times out in
    // fetchStatus and counts as unreachable, so old ages don't look current.
    async function poll(): Promise<void> {
      try {
        status.value = await fetchStatus();
        unreachable.value = false;
      } catch {
        unreachable.value = true;
      }
      if (!stopped) timer = window.setTimeout(poll, POLL_INTERVAL_MS);
    }

    onMounted(() => void poll());
    onUnmounted(() => {
      stopped = true;
      window.clearTimeout(timer);
    });

    const speed = (value: unknown) => formatSpeed(value, unit);

    function age(section: TelemetrySection | null | undefined): { text: string; level: string } {
      if (section?.updated == null || !status.value) return { text: "no data", level: "bad" };
      const seconds = (status.value.server_time - section.updated) / 1000;
      return {
        text: seconds < 1 ? "now" : `${formatAge(Math.round(seconds))} ago`,
        level: seconds > 10 ? "bad" : seconds > 3 ? "warn" : "",
      };
    }

    const KNOWN = new Set(["true_wind", "apparent_wind", "heading", "gps"]);

    function genericValue(value: unknown): string {
      if (typeof value === "number") return String(+value.toFixed(3));
      if (typeof value === "object") return JSON.stringify(value);
      return String(value);
    }

    const cards = computed<Card[]>(() => {
      const s = status.value;
      if (!s) return [];
      const v = s.values;
      const tw = v.true_wind ?? {};
      const aw = v.apparent_wind;
      const hd = v.heading;
      const gps = v.gps;
      const waitingFor = tw.waiting_for as string[] | undefined;
      const rows = (...items: ([string, string | number] | null)[]) =>
        items.filter((item): item is [string, string | number] => item !== null);

      const result: Card[] = [
        {
          title: "True wind",
          section: v.true_wind ?? null,
          rows: rows(
            waitingFor?.length ? ["Waiting for", waitingFor.join(", ")] : null,
            ["Current", tw.speed != null ? `${speed(tw.speed)} from ${formatDirection(tw.direction)}` : "—"],
            [
              "5 s average",
              tw.average_speed != null
                ? `${speed(tw.average_speed)} from ${formatDirection(tw.average_direction)}`
                : "—",
            ],
            ["Skipped, outdated input", (tw.skipped_stale as number | undefined) ?? 0],
          ),
        },
        {
          title: "Apparent wind",
          section: aw ?? null,
          rows: rows(["Speed", speed(aw?.speed)], ["Angle", formatDirection(aw?.angle)]),
        },
        {
          title: "Heading",
          section: hd ?? null,
          rows: rows(
            ["Heading", formatDirection(hd?.heading)],
            ["Compass offset", formatNumber(hd?.compass_offset, 1, "°")],
          ),
        },
        {
          title: "GPS",
          section: gps ?? null,
          rows: rows(
            gps?.waiting_for_fix ? ["Status", "waiting for a fix"] : null,
            ["Fix", gps ? `${gps.fix_type ?? "—"}${gps.valid === false ? " (invalid)" : ""}` : "—"],
            ["Satellites", (gps?.satellites as number | undefined) ?? "—"],
            [
              "Position",
              typeof gps?.latitude === "number" && typeof gps.longitude === "number"
                ? `${gps.latitude.toFixed(5)}, ${gps.longitude.toFixed(5)}`
                : "—",
            ],
            ["Speed over ground", speed(gps?.speed)],
            ["Course over ground", formatDirection(gps?.course)],
            ["HDOP / PDOP", `${formatNumber(gps?.hdop, 2)} / ${formatNumber(gps?.pdop, 2)}`],
            [
              "System clock vs GPS",
              typeof gps?.clock_offset_ms === "number"
                ? // includes the time the position took to arrive, so a few
                  // tenths of a second are normal
                  `${(gps.clock_offset_ms / 1000).toFixed(1)} s` +
                  (Math.abs(gps.clock_offset_ms) < 1000 ? " (normal, includes delivery)" : "")
                : "—",
            ],
          ),
        },
      ];

      // Sections published by other parts of the windbot, e.g. a velocity filter
      for (const [name, values] of Object.entries(v)) {
        if (KNOWN.has(name)) continue;
        result.push({
          title: name.replaceAll("_", " ").replace(/^./, (c) => c.toUpperCase()),
          section: values,
          rows: Object.entries(values)
            .filter(([key]) => key !== "updated")
            .map(([key, value]) => [key.replaceAll("_", " "), genericValue(value)]),
        });
      }

      const { mqtt, system } = s;
      result.push(
        {
          title: "Internet & MQTT",
          rows: rows(
            [
              "MQTT",
              mqtt.enabled ? (mqtt.state === "success" ? "connected" : (mqtt.state ?? "connecting")) : "disabled",
            ],
            mqtt.enabled ? ["Broker", mqtt.broker ?? "—"] : null,
            mqtt.enabled ? ["Connection errors", mqtt.error_count ?? 0] : null,
            ...system.interfaces.map(
              (i): [string, string] => [i.name, `${i.up ? "up" : "down"} ${i.addresses.join(", ")}`],
            ),
          ),
        },
        {
          title: "System",
          rows: rows(
            ["Uptime", system.uptime != null ? formatAge(Math.round(system.uptime)) : "—"],
            ["Load", formatNumber(system.load, 2)],
            ["CPU temperature", formatNumber(system.cpu_temperature, 1, " °C")],
            [
              "Memory free",
              system.memory_available_mb != null
                ? `${system.memory_available_mb} / ${system.memory_total_mb} MB`
                : "—",
            ],
            [
              "Disk free",
              system.disk_free_mb != null ? `${system.disk_free_mb} / ${system.disk_total_mb} MB` : "—",
            ],
            ["Throttled", system.throttled ?? "—"],
            ["Windbot time", new Date(s.server_time).toLocaleString()],
          ),
        },
      );
      return result;
    });

    return { status, unreachable, age, cards };
  },
});
