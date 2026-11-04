import {
  computed,
  defineComponent,
  nextTick,
  onMounted,
  onUnmounted,
  ref,
  watch,
} from "vue";
import { fetchMeasurements, type Measurement as LocalMeasurement } from "./api";
import { LineChart } from "echarts/charts";
import {
  GridComponent,
  MarkLineComponent,
  TooltipComponent,
} from "echarts/components";
import * as echarts from "echarts/core";
import { CanvasRenderer } from "echarts/renderers";

export default defineComponent({
  setup() {
    // Only the parts of ECharts the charts use, which keeps the bundle the Pi serves small
    echarts.use([
      LineChart,
      GridComponent,
      MarkLineComponent,
      TooltipComponent,
      CanvasRenderer,
    ]);

    interface Measurement {
      id: string;
      source_id: string;
      ts: string;
      gps_lat?: number;
      gps_lng?: number;
      sensor_heading_deg?: number;
      sensor_speed_mps?: number;
      true_wind_dir_deg: number;
      true_wind_speed_mps: number;
      apparent_wind_dir_deg?: number;
      apparent_wind_speed_mps?: number;
    }

    type WindSpeedUnit = "mps" | "knots" | "kmh";

    const loading = ref(true);
    const error = ref("");
    const latestMeasurement = ref<Measurement | null>(null);
    const measurements = ref<Measurement[]>([]);
    const lastUpdateTime = ref("");
    const chartsReady = ref(false);
    /** current time on the windbot's clock, which the data ages are measured on */
    const nowMs = ref(Date.now());
    /** windbot clock minus the browser's, from the server_time of the last response */
    let serverOffsetMs = 0;
    const STORAGE_KEY = "windDashboardSettings";
    function loadStoredSettings(): {
      timeFrameMinutes: number;
      windSpeedUnit: WindSpeedUnit;
      sensorSpeedThresholdMps: number;
      directionOldestAtTop: boolean;
      exclusionWindowSec: number;
      hideInvalidData: boolean;
      accelThresholdMs2: number;
      headingRateThresholdDegs: number;
      avgWindowMinutes: number;
      minChartWidthDeg: number;
      maxDisplaySpanDeg: number;
      smoothingWindowSec: number;
    } {
      const defaultSettings = {
        timeFrameMinutes: 30,
        windSpeedUnit: "knots",
        sensorSpeedThresholdMps: 15 / 1.943844,
        directionOldestAtTop: false,
        exclusionWindowSec: 5,
        hideInvalidData: false,
        accelThresholdMs2: 0.3,
        headingRateThresholdDegs: 10,
        avgWindowMinutes: 10,
        minChartWidthDeg: 45,
        maxDisplaySpanDeg: 360,
        smoothingWindowSec: 30,
      } as const;

      try {
        const raw = localStorage.getItem(STORAGE_KEY);
        if (raw) {
          const parsed = JSON.parse(raw) as any;
          return {
            ...defaultSettings,
            ...parsed,
            sensorSpeedThresholdMps:
              parsed.sensorSpeedThresholdMps ??
              parsed.maxSensorSpeedMps ??
              defaultSettings.sensorSpeedThresholdMps,
          };
        }
      } catch {
        /* ignore */
      }
      return defaultSettings;
    }
    const storedSettings = loadStoredSettings();
    const timeFrameMinutes = ref(storedSettings.timeFrameMinutes);
    const settingsOpen = ref(false);
    const pendingTimeFrame = ref(timeFrameMinutes.value);
    const pendingWindSpeedUnit = ref<WindSpeedUnit>(storedSettings.windSpeedUnit);
    const pendingSensorSpeedThreshold = ref(
      roundOne(
        convertWindSpeedFromMps(
          storedSettings.sensorSpeedThresholdMps,
          storedSettings.windSpeedUnit
        )
      )
    );
    const pendingOldestAtTop = ref(storedSettings.directionOldestAtTop);
    const pendingExclusionWindowSec = ref(storedSettings.exclusionWindowSec);
    const pendingHideInvalidData = ref(storedSettings.hideInvalidData);
    const pendingAccelThresholdMs2 = ref(storedSettings.accelThresholdMs2);
    const pendingHeadingRateThresholdDegs = ref(
      storedSettings.headingRateThresholdDegs
    );
    const pendingAvgWindowMinutes = ref(storedSettings.avgWindowMinutes);
    const pendingMinChartWidthDeg = ref(storedSettings.minChartWidthDeg);
    const pendingMaxDisplaySpanDeg = ref(storedSettings.maxDisplaySpanDeg);
    const pendingSmoothing = ref(storedSettings.smoothingWindowSec);

    // Set to true to render newest direction samples at the bottom.
    const windSpeedUnit = ref<WindSpeedUnit>(storedSettings.windSpeedUnit);
    const sensorSpeedThresholdMps = ref(storedSettings.sensorSpeedThresholdMps);
    const directionOldestAtTop = ref(storedSettings.directionOldestAtTop);
    const exclusionWindowSec = ref(storedSettings.exclusionWindowSec);
    const hideInvalidData = ref(storedSettings.hideInvalidData);
    const accelThresholdMs2 = ref(storedSettings.accelThresholdMs2);
    const headingRateThresholdDegs = ref(storedSettings.headingRateThresholdDegs);
    const avgWindowMinutes = ref(storedSettings.avgWindowMinutes);
    const minChartWidthDeg = ref(storedSettings.minChartWidthDeg);
    const maxDisplaySpanDeg = ref(storedSettings.maxDisplaySpanDeg);
    const smoothingWindowSec = ref(storedSettings.smoothingWindowSec);

    const windSpeedUnitLabel = computed(() => {
      if (windSpeedUnit.value === "knots") return "kn";
      if (windSpeedUnit.value === "kmh") return "km/h";
      return "m/s";
    });

    const pendingWindSpeedUnitLabel = computed(() => {
      if (pendingWindSpeedUnit.value === "knots") return "kn";
      if (pendingWindSpeedUnit.value === "kmh") return "km/h";
      return "m/s";
    });

    function roundOne(value: number): number {
      return Math.round(value * 10) / 10;
    }

    function convertWindSpeedFromMps(speedMps: number, unit: WindSpeedUnit): number {
      if (unit === "knots") return speedMps * 1.943844;
      if (unit === "kmh") return speedMps * 3.6;
      return speedMps;
    }

    function convertWindSpeedToMps(speed: number, unit: WindSpeedUnit): number {
      if (unit === "knots") return speed / 1.943844;
      if (unit === "kmh") return speed / 3.6;
      return speed;
    }

    const filteredMeasurements = computed(() =>
      measurements.value.filter(
        (m) =>
          m.sensor_speed_mps == null ||
          m.sensor_speed_mps <= sensorSpeedThresholdMps.value
      )
    );

    const latestDisplayMeasurement = computed<Measurement | null>(
      () => filteredMeasurements.value[0] ?? null
    );

    const latestDataAgeSec = computed(() => {
      if (!latestDisplayMeasurement.value?.ts) return null;
      const ageMs = nowMs.value - new Date(latestDisplayMeasurement.value.ts).getTime();
      return Math.max(0, Math.floor(ageMs / 1000));
    });

    function formatDataAge(ageSec: number): string {
      if (ageSec >= 3600) {
        const hours = Math.floor(ageSec / 3600);
        const minutes = Math.floor((ageSec % 3600) / 60);
        const seconds = ageSec % 60;
        return `${hours}h${String(minutes).padStart(2, "0")}m${String(seconds).padStart(2, "0")}s`;
      }

      if (ageSec >= 60) {
        const minutes = Math.floor(ageSec / 60);
        const seconds = ageSec % 60;
        return `${minutes}m${String(seconds).padStart(2, "0")}s`;
      }

      return `${ageSec}s`;
    }

    const kpiValueAgeClass = computed(() => {
      const ageSec = latestDataAgeSec.value;
      if (ageSec == null) return "";
      if (ageSec > 150) return "kpi-stale-error";
      if (ageSec > 30) return "kpi-stale-warning";
      return "";
    });

    const staleAlertLevel = computed<"warning" | "error" | null>(() => {
      const ageSec = latestDataAgeSec.value;
      if (ageSec == null || ageSec <= 60) return null;
      if (ageSec > 150) return "error";
      return "warning";
    });

    const staleDataNotice = computed(() => {
      const ageSec = latestDataAgeSec.value;
      if (ageSec == null || ageSec <= 60) return "";
      return `Latest data is ${formatDataAge(ageSec)} old.`;
    });

    function convertWindSpeed(speedMps: number): number {
      return convertWindSpeedFromMps(speedMps, windSpeedUnit.value);
    }

    function formatSpeed(speedMps?: number): string {
      if (speedMps == null || !Number.isFinite(speedMps)) return "—";
      return `${convertWindSpeed(speedMps).toFixed(1)} ${windSpeedUnitLabel.value}`;
    }

    function formatDirection(deg?: number): string {
      if (deg == null || !Number.isFinite(deg)) return "—";
      return `${deg.toFixed(0)}°`;
    }

    function getClosestMeasurementByTimeMs(timeMs: number): Measurement | null {
      if (!Number.isFinite(timeMs) || filteredMeasurements.value.length === 0)
        return null;

      let closest: Measurement | null = null;
      let closestDiff = Number.POSITIVE_INFINITY;

      for (const m of filteredMeasurements.value) {
        const mTs = new Date(m.ts).getTime();
        const diff = Math.abs(mTs - timeMs);
        if (diff < closestDiff) {
          closest = m;
          closestDiff = diff;
          if (diff === 0) break;
        }
      }

      return closest;
    }

    function getNearestSpeedPointByTimeMs(
      timeMs: number
    ): { time: string; timeMs: number } | null {
      if (latestSpeedPoints.length === 0) return null;

      let nearest = latestSpeedPoints[0];
      let bestDiff = Math.abs(nearest.timeMs - timeMs);

      for (let i = 1; i < latestSpeedPoints.length; i++) {
        const point = latestSpeedPoints[i];
        const diff = Math.abs(point.timeMs - timeMs);
        if (diff < bestDiff) {
          nearest = point;
          bestDiff = diff;
          if (diff === 0) break;
        }
      }

      return nearest;
    }

    function getTooltipMeasurement(paramsOrParam: any): Measurement | null {
      const params = Array.isArray(paramsOrParam)
        ? paramsOrParam
        : [paramsOrParam];
      const usableParams = params.filter(
        (p) => p && p.componentSubType !== "markLine"
      );
      if (usableParams.length === 0) return null;

      // Resolve time from series payload shape:
      // - direction points: [direction, timeMs]
      // - speed points: [timeMs, speed]
      const timePointParam = usableParams.find(
        (p) => Array.isArray(p.data) && p.data.length >= 2
      );
      if (timePointParam && Array.isArray(timePointParam.data)) {
        if (
          typeof timePointParam.seriesName === "string" &&
          timePointParam.seriesName.startsWith("Wind Speed") &&
          typeof timePointParam.data[0] === "number"
        ) {
          return getClosestMeasurementByTimeMs(timePointParam.data[0]);
        }

        if (typeof timePointParam.data[1] === "number") {
          return getClosestMeasurementByTimeMs(timePointParam.data[1]);
        }
      }

      // Speed chart points align 1:1 with measurements by dataIndex.
      const indexedParam = usableParams.find(
        (p) =>
          typeof p.dataIndex === "number" &&
          p.dataIndex >= 0 &&
          p.dataIndex < filteredMeasurements.value.length
      );
      if (indexedParam) {
        return filteredMeasurements.value[indexedParam.dataIndex];
      }

      // Axis-trigger fallback (for time axis values on direction chart).
      const axisValue = usableParams[0]?.axisValue;
      if (typeof axisValue === "number") {
        return getClosestMeasurementByTimeMs(axisValue);
      }

      return null;
    }

    function getHoveredTimeMsFromTooltipParams(paramsOrParam: any): number | null {
      const measurement = getTooltipMeasurement(paramsOrParam);
      return parseTsMs(measurement?.ts);
    }

    function getHoverSourceFromTooltipParams(
      paramsOrParam: any
    ): "direction" | "speed" | null {
      const params = Array.isArray(paramsOrParam)
        ? paramsOrParam
        : [paramsOrParam];
      const usableParam = params.find((p) => p && p.componentSubType !== "markLine");
      if (!usableParam || typeof usableParam.seriesName !== "string") return null;

      if (usableParam.seriesName.startsWith("Direction")) return "direction";
      if (usableParam.seriesName.startsWith("Wind Speed")) return "speed";
      return null;
    }

    function getDirectionHoverMarkLineData(): Array<Record<string, unknown>> {
      if (hoveredTimeMs == null) return [];
      return [
        {
          yAxis: hoveredTimeMs,
          lineStyle: {
            color: "#2f855a",
            type: "dashed",
            width: 1,
          },
          label: {
            show: false,
          },
        },
      ];
    }

    function getSpeedHoverMarkLineData(): Array<Record<string, unknown>> {
      if (hoveredTimeMs == null) return [];
      return [
        {
          xAxis: hoveredTimeMs,
          lineStyle: {
            color: "#2f855a",
            type: "dashed",
            width: 1,
          },
          label: {
            show: false,
          },
        },
      ];
    }

    function syncHoverTimeLines(): void {
      directionChart?.setOption({
        series: [{}, {}, { markLine: { data: getDirectionHoverMarkLineData() } }],
      });
      speedChart?.setOption({
        series: [{}, {}, { markLine: { data: getSpeedHoverMarkLineData() } }],
      });
    }

    function setHoveredTimeMs(
      timeMs: number | null,
      source: "direction" | "speed" | null = null
    ): void {
      if (hoveredTimeMs === timeMs) return;
      hoveredTimeMs = timeMs;
      if (timeMs == null) {
        hoveredSource = null;
      } else if (source != null) {
        hoveredSource = source;
      }
      syncHoverTimeLines();
    }

    function clearHoverFromSource(source: "direction" | "speed"): void {
      if (hoveredSource === source) {
        setHoveredTimeMs(null);
      }
    }

    function sensorTooltipLines(measurement: Measurement): string[] {
      return [
        `Sensor speed: ${formatSpeed(measurement.sensor_speed_mps)}`,
        `Sensor direction: ${formatDirection(measurement.sensor_heading_deg)}`,
      ];
    }

    function buildMeasurementTooltip(paramsOrParam: any): string {
      setHoveredTimeMs(
        getHoveredTimeMsFromTooltipParams(paramsOrParam),
        getHoverSourceFromTooltipParams(paramsOrParam)
      );
      const measurement = getTooltipMeasurement(paramsOrParam);
      if (!measurement) return "No measurement available";

      const ts = new Date(measurement.ts).toLocaleTimeString();
      return [
        `<strong>${ts}</strong>`,
        `True wind speed: ${formatSpeed(measurement.true_wind_speed_mps)}`,
        `True wind direction: ${formatDirection(measurement.true_wind_dir_deg)}`,
        ...sensorTooltipLines(measurement),
        `Apparent wind speed: ${formatSpeed(measurement.apparent_wind_speed_mps)}`,
        `Apparent wind direction: ${formatDirection(measurement.apparent_wind_dir_deg)}`,
      ].join("<br/>");
    }

    // Human-readable label shown in the KPI cards
    const avgWindow = computed(() => `${avgWindowMinutes.value} min`);

    const averageDirection = (directions: number[]) => {
      // Circular mean for direction
      const sinSum = directions.reduce((s, d) => s + Math.sin((d * Math.PI) / 180), 0);
      const cosSum = directions.reduce((s, d) => s + Math.cos((d * Math.PI) / 180), 0);
      return ((Math.atan2(sinSum / directions.length, cosSum / directions.length) * 180) / Math.PI + 360) % 360;
    };

    // Rolling average smoothing helpers.
    // measurements is sorted newest-first; windowSamples = smoothingWindowSec - 4
    // (accounts for the 5 s pre-averaging already applied to incoming data).
    function smoothDirections(points: { direction: number; timeMs: number }[], windowSamples: number): { direction: number; timeMs: number }[] {
      if (windowSamples <= 1) return points;
      return points.map((_, i) => {
        const slice = points.slice(i, i + windowSamples);
        return { direction: averageDirection(slice.map((p) => p.direction)), timeMs: points[i].timeMs };
      });
    }

    function smoothSpeeds(points: { speed: number; time: string; timeMs: number }[], windowSamples: number): { speed: number; time: string; timeMs: number }[] {
      if (windowSamples <= 1) return points;
      return points.map((_, i) => {
        const slice = points.slice(i, i + windowSamples);
        const avg = slice.reduce((s, p) => s + p.speed, 0) / slice.length;
        return { speed: avg, time: points[i].time, timeMs: points[i].timeMs };
      });
    }

    const smoothingSamples = computed(() => Math.max(1, smoothingWindowSec.value - 4));

    // Average of true_wind_dir_deg and true_wind_speed_mps over the last avgWindowMinutes
    const averageMeasurement = computed(() => {
      if (!latestDisplayMeasurement.value || filteredMeasurements.value.length === 0)
        return null;
      const cutoff =
        new Date(latestDisplayMeasurement.value.ts).getTime() -
        avgWindowMinutes.value * 60 * 1000;
      const window = filteredMeasurements.value.filter(
        (m) => new Date(m.ts).getTime() >= cutoff
      );
      if (window.length === 0) return null;

      const unreliableRanges = computeUnreliableRanges(
        window,
        exclusionWindowSec.value * 1000
      );
      const reliableWindow = window.filter((m) => {
        const ts = new Date(m.ts).getTime();
        for (const [start, end] of unreliableRanges) {
          if (ts >= start && ts <= end) return false;
        }
        return true;
      });

      if (reliableWindow.length === 0) return null;

      const avgSpeed =
        reliableWindow.reduce((s, m) => s + m.true_wind_speed_mps, 0) /
        reliableWindow.length;
      const avgDir = averageDirection(
        reliableWindow.map((m) => m.true_wind_dir_deg)
      );
      return { true_wind_speed_mps: avgSpeed, true_wind_dir_deg: avgDir };
    });

    // Detect sensor acceleration / heading-rate events and return merged unreliable time ranges.
    function computeUnreliableRanges(
      meas: Measurement[],
      windowMs: number
    ): Array<[number, number]> {
      if (windowMs <= 0) return [];
      const eventTimes: number[] = [];
      for (let i = 0; i < meas.length - 1; i++) {
        const a = meas[i];
        const b = meas[i + 1];
        const dtMs = Math.abs(new Date(a.ts).getTime() - new Date(b.ts).getTime());
        if (dtMs === 0) continue;
        const dtS = dtMs / 1000;
        const midTime = (new Date(a.ts).getTime() + new Date(b.ts).getTime()) / 2;
        if (a.sensor_speed_mps != null && b.sensor_speed_mps != null) {
          if (
            Math.abs(a.sensor_speed_mps - b.sensor_speed_mps) / dtS >
            accelThresholdMs2.value
          )
            eventTimes.push(midTime);
        }
        if (a.sensor_heading_deg != null && b.sensor_heading_deg != null) {
          const avgSpeed = ((a.sensor_speed_mps ?? 0) + (b.sensor_speed_mps ?? 0)) / 2;
          if (avgSpeed > 1) {
            let dh = Math.abs(a.sensor_heading_deg - b.sensor_heading_deg);
            if (dh > 180) dh = 360 - dh;
            if (dh / dtS > headingRateThresholdDegs.value) eventTimes.push(midTime);
          }
        }
      }
      const raw = eventTimes.map(
        (t) => [t - windowMs, t + windowMs] as [number, number]
      );
      raw.sort((a, b) => a[0] - b[0]);
      const merged: Array<[number, number]> = [];
      for (const r of raw) {
        if (merged.length > 0 && r[0] <= merged[merged.length - 1][1])
          merged[merged.length - 1][1] = Math.max(
            merged[merged.length - 1][1],
            r[1]
          );
        else merged.push([...r] as [number, number]);
      }
      return merged;
    }

    async function applySettings() {
      const prevTimeFrameMinutes = timeFrameMinutes.value;

      timeFrameMinutes.value = pendingTimeFrame.value;
      windSpeedUnit.value = pendingWindSpeedUnit.value;
      sensorSpeedThresholdMps.value = Math.max(
        0,
        convertWindSpeedToMps(
          Number.isFinite(pendingSensorSpeedThreshold.value)
            ? pendingSensorSpeedThreshold.value
            : 0,
          pendingWindSpeedUnit.value
        )
      );
      directionOldestAtTop.value = pendingOldestAtTop.value;
      exclusionWindowSec.value = pendingExclusionWindowSec.value;
      hideInvalidData.value = pendingHideInvalidData.value;
      accelThresholdMs2.value = pendingAccelThresholdMs2.value;
      headingRateThresholdDegs.value = pendingHeadingRateThresholdDegs.value;
      avgWindowMinutes.value = pendingAvgWindowMinutes.value;
      minChartWidthDeg.value = pendingMinChartWidthDeg.value;
      maxDisplaySpanDeg.value = pendingMaxDisplaySpanDeg.value;
      smoothingWindowSec.value = pendingSmoothing.value;
      settingsOpen.value = false;

      // Persist to local storage
      localStorage.setItem(
        STORAGE_KEY,
        JSON.stringify({
          timeFrameMinutes: timeFrameMinutes.value,
          windSpeedUnit: windSpeedUnit.value,
          sensorSpeedThresholdMps: sensorSpeedThresholdMps.value,
          directionOldestAtTop: directionOldestAtTop.value,
          exclusionWindowSec: exclusionWindowSec.value,
          hideInvalidData: hideInvalidData.value,
          accelThresholdMs2: accelThresholdMs2.value,
          headingRateThresholdDegs: headingRateThresholdDegs.value,
          avgWindowMinutes: avgWindowMinutes.value,
          minChartWidthDeg: minChartWidthDeg.value,
          maxDisplaySpanDeg: maxDisplaySpanDeg.value,
          smoothingWindowSec: smoothingWindowSec.value,
        })
      );

      // Dispose existing charts so they reinitialize cleanly
      chartsReady.value = false;
      directionChart?.dispose();
      speedChart?.dispose();
      directionChart = null;
      speedChart = null;

      await nextTick();
      initCharts();
      chartsReady.value = true;

      if (timeFrameMinutes.value > prevTimeFrameMinutes) {
        // Only fetch more data when the requested window grew.
        await loadMeasurements();
        return;
      }

      if (latestMeasurement.value?.ts) {
        const newestTsMs = new Date(latestMeasurement.value.ts).getTime();
        const cutoffMs = newestTsMs - timeFrameMinutes.value * 60 * 1000;
        measurements.value = measurements.value.filter(
          (m) => new Date(m.ts).getTime() > cutoffMs
        );
      }

      updateCharts();
      updateTime();
    }

    let directionChart: echarts.ECharts | null = null;
    let speedChart: echarts.ECharts | null = null;
    let hoveredTimeMs: number | null = null;
    let hoveredSource: "direction" | "speed" | null = null;
    let latestSpeedPoints: Array<{ time: string; timeMs: number }> = [];
    let staleAgeTimerId: number | null = null;
    let pollTimerId: number | null = null;
    let runningPoll: Promise<void> | null = null;

    const POLL_INTERVAL_MS = 1000;
    /** ms until the first request, which fetches the whole history, counts as failed */
    const HISTORY_TIMEOUT_MS = 60_000;

    function fail(reason: string): never {
      throw new Error(reason);
    }

    function parseTsMs(ts?: string): number | null {
      if (!ts) return null;
      const ms = new Date(ts).getTime();
      return Number.isFinite(ms) ? ms : null;
    }

    // The windbot serves one second records with millisecond timestamps, see
    // src/dashboard/history.py. They get the shape of the online measurements.
    function toMeasurement(record: LocalMeasurement): Measurement {
      return {
        ...record,
        id: String(record.ts),
        source_id: "local",
        ts: new Date(record.ts).toISOString(),
        gps_lat: record.gps_lat ?? undefined,
        gps_lng: record.gps_lng ?? undefined,
        sensor_heading_deg: record.sensor_heading_deg ?? undefined,
        sensor_speed_mps: record.sensor_speed_mps ?? undefined,
      };
    }

    // Fetches the records newer than the latest one (those of the time frame on the
    // first call). Only one request runs at a time; callers share the running one.
    function pollMeasurements(): Promise<void> {
      runningPoll ??= fetchNewMeasurements().finally(() => {
        runningPoll = null;
      });
      return runningPoll;
    }

    async function fetchNewMeasurements() {
      try {
        const since = parseTsMs(latestMeasurement.value?.ts) ?? undefined;
        // The history can take a while on a Pi Zero and a weak hotspot
        const data = await fetchMeasurements(
          since,
          since === undefined ? timeFrameMinutes.value * 60 : undefined,
          since === undefined ? HISTORY_TIMEOUT_MS : undefined
        );
        serverOffsetMs = data.server_time - Date.now();
        nowMs.value = data.server_time;
        error.value = "";
        if (data.items.length > 0) {
          // newest first, like the online dashboard
          const fresh = data.items.map(toMeasurement).reverse();
          const cutoffMs =
            new Date(fresh[0].ts).getTime() - timeFrameMinutes.value * 60 * 1000;
          measurements.value = [...fresh, ...measurements.value].filter(
            (m) => new Date(m.ts).getTime() > cutoffMs
          );
          latestMeasurement.value = measurements.value[0] ?? null;
          updateCharts();
          updateTime();
        }
      } catch (e) {
        error.value = "The windbot can't be reached.";
        console.error("Failed to load measurements:", e);
      } finally {
        loading.value = false;
      }
    }

    // Reloads everything, e.g. when the time frame grew. A poll that is still running
    // would put only its few new records into the emptied list, so it is waited for.
    async function loadMeasurements() {
      await runningPoll;
      measurements.value = [];
      latestMeasurement.value = null;
      await pollMeasurements();
    }

    function handleTabRefocus() {
      if (document.hidden) return;
      void pollMeasurements();
    }

    function updateTime() {
      if (latestDisplayMeasurement.value?.ts) {
        const date = new Date(latestDisplayMeasurement.value.ts);
        lastUpdateTime.value = date.toLocaleTimeString();
      }
    }

    function updateCharts() {
      if (!chartsReady.value) {
        return;
      }

      const visibleMeasurements = filteredMeasurements.value;

      if (visibleMeasurements.length === 0) {
        console.log("No measurements to chart");
        return;
      }

      // Wind direction chart (x = direction, y = time)
      const rawDirectionPoints = visibleMeasurements.map((m) => ({
        direction: m.true_wind_dir_deg,
        timeMs: new Date(m.ts).getTime(),
      }));
      const directionPoints = smoothDirections(rawDirectionPoints, smoothingSamples.value);

      // Compute unreliable ranges once for use in both charts
      const unreliableRanges = computeUnreliableRanges(
        visibleMeasurements,
        exclusionWindowSec.value * 1000
      );

      function isUnreliableTs(timeMs: number): boolean {
        for (const [start, end] of unreliableRanges) {
          if (timeMs >= start && timeMs <= end) return true;
        }
        return false;
      }

      if (directionChart && directionPoints.length > 0) {
        // Filter out unreliable points for average calculation
        const reliableDirectionPoints = directionPoints.filter((p) => !isUnreliableTs(p.timeMs));

        // Calculate average wind direction (excluding unreliable samples)
        const avgDirection = reliableDirectionPoints.length > 0 ?
          averageDirection(reliableDirectionPoints.map((dp) => dp.direction))
          : directionPoints.length > 0 ? averageDirection(directionPoints.map((dp) => dp.direction)) : 0;

        // Compute displaySpan based on largest distance from average
        // When hiding invalid data, only use reliable points for span calculation
        const pointsForSpan = hideInvalidData.value ? reliableDirectionPoints : directionPoints;
        let maxDistance = 0;
        for (const point of pointsForSpan) {
          let distance = Math.abs(point.direction - avgDirection);
          // Handle wrap-around: the shortest distance might be the other way
          if (distance > 180) {
            distance = 360 - distance;
          }
          maxDistance = Math.max(maxDistance, distance);
        }

        // Use minimum chart width setting, but cap at user-selected maximum display span.
        const displaySpan = Math.min(maxDisplaySpanDeg.value / 2, Math.max(minChartWidthDeg.value / 2, maxDistance + 5));
        let axisMin = avgDirection - displaySpan;
        let axisMax = avgDirection + displaySpan;

        // Normalize data points for display with wrap-around
        const normalizedPoints = directionPoints.map((p) => {
          let value = p.direction;
          // Shift points to the correct side of the axis if they wrap
          if (value < axisMin) {
            value += 360;
          } else if (value > axisMax) {
            value -= 360;
          }
          return { ...p, direction: value };
        });

        // Round axis bounds to nearest 5 for clean labels
        axisMin = Math.round(axisMin / 5) * 5;
        axisMax = Math.round(axisMax / 5) * 5;

        // Build two data arrays:
        // - reliableSeriesData: solid line
        // - excludedSeriesData: dashed line for ignored windows
        const reliableSeriesData: Array<[number, number] | [null, null]> = [];
        const excludedSeriesData: Array<[number, number] | [null, null]> = [];
        const eps = 1e-6;


        function pushPoint(
          direction: number,
          timeMs: number,
          unreliable: boolean
        ): void {
          if (unreliable) {
            reliableSeriesData.push([null, null]);
            excludedSeriesData.push([direction, timeMs]);
          } else {
            reliableSeriesData.push([direction, timeMs]);
            excludedSeriesData.push([null, null]);
          }
        }

        // Keep track of the last appended sample so dashed segments can include
        // copied valid endpoints at reliability transitions.
        let hasLastSample = false;
        let lastDirection = 0;
        let lastTimeMs = 0;
        let lastUnreliable = false;

        function appendSample(
          direction: number,
          timeMs: number,
          unreliable: boolean
        ): void {
          if (hasLastSample && lastUnreliable !== unreliable) {
            if (unreliable) {
              // Entering excluded region: copy the last reliable point into dashed series.
              pushPoint(lastDirection, lastTimeMs, true);
            } else {
              // Leaving excluded region: copy the current reliable point into dashed series.
              pushPoint(direction, timeMs, true);
            }
          }

          pushPoint(direction, timeMs, unreliable);
          hasLastSample = true;
          lastDirection = direction;
          lastTimeMs = timeMs;
          lastUnreliable = unreliable;
        }

        function pushBreak(): void {
          reliableSeriesData.push([null, null]);
          excludedSeriesData.push([null, null]);
          hasLastSample = false;
        }

        // Build series from normalized values only.
        // If the normalized gap exceeds 180 deg, force a cutoff break.
        for (let i = 0; i < normalizedPoints.length; ++i) {
          const point = normalizedPoints[i];
          const pointUnreliable = isUnreliableTs(point.timeMs);
          appendSample(point.direction, point.timeMs, pointUnreliable);

          if (i >= normalizedPoints.length - 1) continue;

          const nextPoint = normalizedPoints[i + 1];
          const gap = Math.abs(nextPoint.direction - point.direction);
          if (gap <= 180) continue;

          // Crossing side rule from normalized space:
          // current < avg => cross left cutoff first, current > avg => cross right cutoff first
          const crossesLeftFirst = point.direction < avgDirection;
          const beforeCutoff = crossesLeftFirst ? axisMin + eps : axisMax - eps;
          const afterCutoff = crossesLeftFirst ? axisMax - eps : axisMin + eps;
          const cutTimeBefore = point.timeMs + 0.499;
          const cutTimeAfter = point.timeMs + 0.501;
          const beforeUnreliable = isUnreliableTs(cutTimeBefore);
          const afterUnreliable = isUnreliableTs(cutTimeAfter);

          appendSample(beforeCutoff, cutTimeBefore, beforeUnreliable);
          pushBreak();
          appendSample(afterCutoff, cutTimeAfter, afterUnreliable);
        }

        // Log using original sample count
        console.log(
          "Updating direction chart with",
          directionPoints.length,
          "points, average:",
          avgDirection.toFixed(1)
        );
        try {
          directionChart.setOption({
            animation: false,
            animationDuration: 0,
            animationDurationUpdate: 0,
            xAxis: [
              {
                min: axisMin,
                max: axisMax,
                axisPointer: {
                  label: {
                    formatter: (params: any) => {
                      const value = Number(params?.value);
                      const normalized = ((value % 360) + 360) % 360;
                      return `${Math.round(normalized)}`;
                    },
                  },
                },
              },
              {
                min: Math.floor(axisMin - avgDirection),
                max: Math.ceil(axisMax - avgDirection),
                axisPointer: {
                  label: {
                    formatter: (params: any) => {
                      const value = Number(params?.value);
                      const rounded = Math.round(value);
                      if (rounded >= 0) return `+${rounded}`;
                      return `${rounded}`;
                    },
                  },
                },
                splitLine: {
                  lineStyle: {
                    type: [5, 10],
                  },
                },
              },
            ],
            yAxis: {
              type: "time",
              inverse: directionOldestAtTop.value,
              axisLabel: {
                formatter: (value: number) => new Date(value).toLocaleTimeString(),
              },
            },
            series: [
              {
                name: "Direction (Reliable)",
                data: reliableSeriesData,
                markLine: {
                  symbol: directionOldestAtTop.value
                    ? ["none", "arrow"]
                    : ["arrow", "none"],
                  data: [
                    {
                      name: "Average",
                      xAxis: avgDirection,
                      lineStyle: {
                        color: "#667eea",
                        type: "dashed",
                        width: 2,
                      },
                      label: {
                        position: directionOldestAtTop.value ? "end" : "start",
                        formatter: `Avg: ${avgDirection.toFixed(1)}°`,
                      },
                    },
                  ],
                },
              },
              {
                name: "Direction (Excluded)",
                show: !hideInvalidData.value,
                data: hideInvalidData.value ? [] : excludedSeriesData,
                lineStyle: {
                  type: "dashed",
                  width: 2,
                  color: "#667eea",
                  opacity: 0.9,
                },
              },
              {
                name: "Direction (Hover Time)",
                data: [],
                markLine: {
                  symbol: ["none", "none"],
                  silent: true,
                  data: getDirectionHoverMarkLineData(),
                },
                lineStyle: {
                  opacity: 0,
                },
              },
            ],
          });
          console.log("✓ Direction chart updated");
        } catch (e) {
          console.error("Failed to update direction chart:", e);
        }
      } else {
        console.log(
          "Direction chart not ready or no data:",
          !!directionChart,
          directionPoints.length
        );
      }

      // Wind speed chart
      const rawSpeedPoints = visibleMeasurements.map((m) => ({
        speed: convertWindSpeed(m.true_wind_speed_mps),
        time: new Date(m.ts).toLocaleTimeString(),
        timeMs: new Date(m.ts).getTime(),
      }));
      const speedPoints = smoothSpeeds(rawSpeedPoints, smoothingSamples.value);
      latestSpeedPoints = speedPoints.map((p) => ({ time: p.time, timeMs: p.timeMs }));

      if (speedChart && speedPoints.length > 0) {
        const reliableSpeedData = speedPoints.map((p) =>
          isUnreliableTs(p.timeMs) ? null : [p.timeMs, p.speed]
        );
        const excludedSpeedData = speedPoints.map((p) =>
          isUnreliableTs(p.timeMs) ? [p.timeMs, p.speed] : null
        );
        console.log(
          "Updating speed chart with",
          speedPoints.length,
          "points, first 3:",
          speedPoints.slice(0, 3)
        );
        try {
          speedChart.setOption({
            animation: false,
            animationDuration: 0,
            animationDurationUpdate: 0,
            xAxis: {
              type: "time",
              name: "Time",
              nameLocation: "middle",
              nameTextStyle: {
                padding: [8, 0, 0, 0],
              },
              axisLabel: {
                formatter: (value: number) => new Date(value).toLocaleTimeString(),
              },
            },
            yAxis: {
              type: "value",
              name: `Speed (${windSpeedUnitLabel.value})`,
              nameLocation: "middle",
              nameTextStyle: {
                padding: [0, 0, 12, 0],
              },
            },
            series: [
              {
                name: "Wind Speed (Reliable)",
                data: reliableSpeedData,
              },
              {
                name: "Wind Speed (Excluded)",
                show: !hideInvalidData.value,
                data: hideInvalidData.value ? [] : excludedSpeedData,
              },
              {
                name: "Wind Speed (Hover Time)",
                data: [],
                markLine: {
                  symbol: ["none", "none"],
                  silent: true,
                  data: getSpeedHoverMarkLineData(),
                },
                lineStyle: {
                  opacity: 0,
                },
              },
            ],
          });
          console.log("✓ Speed chart updated");
        } catch (e) {
          console.error("Failed to update speed chart:", e);
        }
      } else {
        console.log(
          "Speed chart not ready or no data:",
          !!speedChart,
          speedPoints.length
        );
      }
    }

    function initCharts() {
      const dirContainer =
        document.getElementById("wind-direction-chart") ??
        fail("Direction chart container not found");
      const speedContainer =
        document.getElementById("wind-speed-chart") ??
        fail("Speed chart container not found");

      // Wind direction chart
      directionChart = echarts.init(dirContainer);
      directionChart.on("globalout", () => {
        clearHoverFromSource("direction");
      });
      directionChart.setOption({
        animation: false,
        animationDuration: 0,
        animationDurationUpdate: 0,
        tooltip: {
          trigger: "axis",
          axisPointer: {
            type: "cross",
            snap: true,
          },
          formatter: (params: any) => buildMeasurementTooltip(params),
        },
        grid: {
          left: "24px",
          right: "16px",
          bottom: "28px",
          top: "28px",
          containLabel: true,
        },
        xAxis: [
          {
            type: "value",
            name: "Direction (deg)",
            nameLocation: "middle",
            nameTextStyle: {
              padding: [0, 0, 8, 0],
            },
            min: 0,
            max: 360,
            position: "top",
            axisLabel: {
              formatter: (value: number) => {
                const normalized = ((value % 360) + 360) % 360;
                return `${Math.round(normalized)}`;
              },
            },
            axisPointer: {
              label: {
                formatter: (params: any) => {
                  const value = Number(params?.value);
                  const normalized = ((value % 360) + 360) % 360;
                  return `${Math.round(normalized)}`;
                },
              },
            },
          },
          {
            type: "value",
            name: "Offset to Avg (deg)",
            nameLocation: "middle",
            nameTextStyle: {
              padding: [8, 0, 0, 0],
            },
            min: -180,
            max: 180,
            position: "bottom",
            axisLabel: {
              formatter: (value: number) => {
                const rounded = Math.round(value);
                if (rounded === 0) {
                  return "0";
                }
                if (rounded > 0) {
                  return `+${rounded}`;
                }
                return `${rounded}`;
              },
            },
            axisPointer: {
              label: {
                formatter: (params: any) => {
                  const value = Number(params?.value);
                  const rounded = Math.round(value);
                  if (rounded >= 0) return `+${rounded}`;
                  return `${rounded}`;
                },
              },
            },
          },
        ],
        yAxis: {
          type: "time",
          name: "Time",
          nameLocation: "middle",
          nameTextStyle: {
            padding: [0, 0, 48, 0],
          },
          inverse: directionOldestAtTop.value,
          axisLabel: {
            inside: false,
            margin: 8,
            padding: [8, 0, 0, 0],
            formatter: (value: number) => new Date(value).toLocaleTimeString(),
          },
          axisLine: {
            show: false,
          },
          axisTick: {
            show: false,
          },
        },
        series: [
          {
            name: "Direction (Reliable)",
            type: "line",
            smooth: false,
            symbol: "circle",
            symbolSize: 5,
            showSymbol: false,
            encode: { x: 0, y: 1 },
            data: [],
            animation: false,
            itemStyle: { color: "#667eea" },
          },
          {
            name: "Direction (Excluded)",
            type: "line",
            smooth: false,
            symbol: "circle",
            symbolSize: 5,
            showSymbol: false,
            encode: { x: 0, y: 1 },
            data: [],
            animation: false,
            lineStyle: {
              type: "dashed",
              width: 2,
              color: "#667eea",
              opacity: 0.9,
            },
          },
          {
            name: "Direction (Hover Time)",
            type: "line",
            data: [],
            symbol: "none",
            lineStyle: {
              opacity: 0,
            },
            markLine: {
              symbol: ["none", "none"],
              silent: true,
              data: [],
            },
          },
        ],
      });
      console.log("✓ Direction chart initialized");

      // Wind speed chart
      speedChart = echarts.init(speedContainer);
      speedChart.on("globalout", () => {
        clearHoverFromSource("speed");
      });
      speedChart.setOption({
        animation: false,
        animationDuration: 0,
        animationDurationUpdate: 0,
        tooltip: {
          trigger: "axis",
          axisPointer: {
            type: "line",
            snap: true,
          },
          formatter: (params: any) => buildMeasurementTooltip(params),
        },
        grid: {
          left: "24px",
          right: "16px",
          bottom: "16px",
          top: "16px",
          containLabel: true,
        },
        xAxis: {
          type: "time",
          name: "Time",
          nameLocation: "middle",
          nameTextStyle: {
            padding: [8, 0, 0, 0],
          },
          axisLabel: {
            formatter: (value: number) => new Date(value).toLocaleTimeString(),
          },
        },
        yAxis: {
          type: "value",
          name: `Speed (${windSpeedUnitLabel.value})`,
          nameLocation: "middle",
          nameTextStyle: {
            padding: [0, 0, 12, 0],
          },
        },
        series: [
          {
            name: "Wind Speed (Reliable)",
            type: "line",
            data: [],
            animation: false,
            smooth: true,
            symbol: "circle",
            symbolSize: 4,
            showSymbol: false,
            itemStyle: {
              color: "#764ba2",
            },
            /*areaStyle: {
              color: 'rgba(118, 75, 162, 0.1)',
            },*/
          },
          {
            name: "Wind Speed (Excluded)",
            type: "line",
            data: [],
            animation: false,
            smooth: true,
            symbol: "circle",
            symbolSize: 4,
            showSymbol: false,
            itemStyle: {
              color: "#764ba2",
            },
            lineStyle: {
              type: "dashed",
              width: 2,
              opacity: 0.9,
            },
          },
          {
            name: "Wind Speed (Hover Time)",
            type: "line",
            data: [],
            symbol: "none",
            lineStyle: {
              opacity: 0,
            },
            markLine: {
              symbol: ["none", "none"],
              silent: true,
              data: [],
            },
          },
        ],
      });
      console.log("✓ Speed chart initialized");

      // Handle window resize
      window.addEventListener("resize", () => {
        directionChart?.resize();
        speedChart?.resize();
      });
    }

    onMounted(() => {
      document.addEventListener("visibilitychange", handleTabRefocus);
      window.addEventListener("focus", handleTabRefocus);
      staleAgeTimerId = window.setInterval(() => {
        nowMs.value = Date.now() + serverOffsetMs;
      }, 1000);
      void pollMeasurements();
      pollTimerId = window.setInterval(pollMeasurements, POLL_INTERVAL_MS);
    });

    onUnmounted(() => {
      document.removeEventListener("visibilitychange", handleTabRefocus);
      window.removeEventListener("focus", handleTabRefocus);
      if (staleAgeTimerId != null) {
        window.clearInterval(staleAgeTimerId);
        staleAgeTimerId = null;
      }
      if (pollTimerId != null) {
        window.clearInterval(pollTimerId);
        pollTimerId = null;
      }
    });

    // Initialize charts once content becomes visible
    watch(loading, async (isLoading) => {
      if (!isLoading) {
        console.log("Content visible, initializing charts...");
        await nextTick();
        initCharts();
        chartsReady.value = true;
        updateCharts();
      }
    });

    watch(pendingWindSpeedUnit, (newUnit, oldUnit) => {
      if (!settingsOpen.value) return;
      const prevMps = convertWindSpeedToMps(
        pendingSensorSpeedThreshold.value,
        oldUnit
      );
      pendingSensorSpeedThreshold.value = roundOne(
        convertWindSpeedFromMps(prevMps, newUnit)
      );
    });

    return {
      loading,
      error,
      timeFrameMinutes,
      settingsOpen,
      pendingTimeFrame,
      pendingWindSpeedUnit,
      pendingSensorSpeedThreshold,
      pendingOldestAtTop,
      pendingExclusionWindowSec,
      pendingHideInvalidData,
      pendingAccelThresholdMs2,
      pendingHeadingRateThresholdDegs,
      pendingAvgWindowMinutes,
      pendingMinChartWidthDeg,
      pendingMaxDisplaySpanDeg,
      pendingSmoothing,
      windSpeedUnit,
      sensorSpeedThresholdMps,
      directionOldestAtTop,
      exclusionWindowSec,
      hideInvalidData,
      accelThresholdMs2,
      headingRateThresholdDegs,
      avgWindowMinutes,
      minChartWidthDeg,
      maxDisplaySpanDeg,
      smoothingWindowSec,
      windSpeedUnitLabel,
      pendingWindSpeedUnitLabel,
      roundOne,
      convertWindSpeedFromMps,
      latestDisplayMeasurement,
      kpiValueAgeClass,
      staleAlertLevel,
      staleDataNotice,
      convertWindSpeed,
      avgWindow,
      averageMeasurement,
      applySettings,
    };
  },
});
