// Placeholder until the charts of the online dashboard are ported: shows the
// latest one second record of /api/measurements.
import { computed, defineComponent, onMounted, onUnmounted, ref } from "vue";
import { fetchMeasurements, type Measurement } from "./api";

type Key = keyof Omit<Measurement, "ts">;

const ROWS: { key: Key; label: string; unit: string; digits: number }[] = [
  { key: "true_wind_speed_mps", label: "True wind speed", unit: " m/s", digits: 1 },
  { key: "true_wind_dir_deg", label: "True wind direction", unit: "°", digits: 1 },
  { key: "apparent_wind_speed_mps", label: "Apparent wind speed", unit: " m/s", digits: 1 },
  { key: "apparent_wind_dir_deg", label: "Apparent wind direction", unit: "°", digits: 1 },
  { key: "sensor_speed_mps", label: "Sensor speed", unit: " m/s", digits: 1 },
  { key: "sensor_heading_deg", label: "Course over ground", unit: "°", digits: 1 },
  { key: "gps_lat", label: "Latitude", unit: "", digits: 5 },
  { key: "gps_lng", label: "Longitude", unit: "", digits: 5 },
];

/** ms between the end of one request and the start of the next */
const POLL_INTERVAL_MS = 1000;
/** ms after which the shown values count as outdated */
const STALE_MS = 10_000;

export default defineComponent({
  setup() {
    const latest = ref<Measurement | null>(null);
    const unreachable = ref(false);
    /** age of the latest record in ms, on the windbot's clock */
    const age = ref(0);
    let timer: number | undefined;
    let stopped = false;

    // One request at a time, so they can't pile up on a slow Pi or arrive out of
    // order. Only the newest record is shown, so none of the history is fetched.
    // A stalled request times out in fetchMeasurements and counts as unreachable.
    async function poll(): Promise<void> {
      try {
        const data = await fetchMeasurements(latest.value?.ts, 1);
        const last = data.items.at(-1);
        if (last) latest.value = last;
        if (latest.value) age.value = data.server_time - latest.value.ts;
        unreachable.value = false;
      } catch {
        unreachable.value = true;
      }
      if (!stopped) timer = window.setTimeout(poll, POLL_INTERVAL_MS);
    }

    const stale = computed(() => unreachable.value || age.value > STALE_MS);

    const state = computed(() => {
      if (unreachable.value) return "The windbot can't be reached. The values are outdated.";
      if (!latest.value) return "Waiting for wind data…";
      const time = new Date(latest.value.ts).toLocaleTimeString();
      if (stale.value) return `No new data since ${time}. The values are outdated.`;
      return `Latest data from ${time}`;
    });

    function format(row: (typeof ROWS)[number]): string {
      const value = latest.value?.[row.key];
      if (value == null) return "—";
      return `${+value.toFixed(row.digits)}${row.unit}`;
    }

    onMounted(() => void poll());
    onUnmounted(() => {
      stopped = true;
      window.clearTimeout(timer);
    });

    return { ROWS, latest, stale, state, format };
  },
});
