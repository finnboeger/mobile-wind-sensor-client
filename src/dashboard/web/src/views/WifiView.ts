import { computed, defineComponent, onMounted, onUnmounted, ref } from "vue";
import {
  fetchWifi,
  postJson,
  scanWifi,
  type KnownNetwork,
  type ScannedNetwork,
  type WifiStatus,
} from "../api";
import { formatAge } from "../format";

export default defineComponent({
  setup() {
    const wifi = ref<WifiStatus | null>(null);
    const wifiError = ref<string | null>(null);
    const scanned = ref<ScannedNetwork[] | null>(null);
    const scanning = ref(false);
    const scanError = ref<string | null>(null);
    /** ms between the end of one request and the start of the next */
    const POLL_INTERVAL_MS = 3000;
    let timer: number | undefined;
    let stopped = false;
    let running: Promise<void> | null = null;

    // One request at a time, like on the other tabs; a stalled one times out.
    function poll(): Promise<void> {
      running ??= refresh().finally(() => {
        running = null;
        window.clearTimeout(timer);
        if (!stopped) timer = window.setTimeout(poll, POLL_INTERVAL_MS);
      });
      return running;
    }

    async function refresh(): Promise<void> {
      try {
        wifi.value = await fetchWifi();
        wifiError.value = null;
      } catch (e) {
        wifiError.value = e instanceof Error ? e.message : String(e);
      }
    }

    async function scan(): Promise<void> {
      scanning.value = true;
      scanError.value = null;
      try {
        scanned.value = (await scanWifi()).networks;
      } catch (e) {
        scanError.value = e instanceof Error ? e.message : String(e);
      } finally {
        scanning.value = false;
      }
    }

    const busy = computed(() => wifi.value?.action != null);

    async function wifiAction(path: string, body: unknown, warning: string | null): Promise<void> {
      if (warning && !confirm(warning)) return;
      try {
        await postJson(path, body);
        wifiError.value = null;
      } catch (e) {
        wifiError.value = e instanceof Error ? e.message : String(e);
      }
      // A request that started before the action may answer with the old state,
      // so wait for it and then ask again
      await running;
      await poll();
    }

    const LOSE_CONNECTION =
      "If you are connected through the hotspot, this page loses its connection. " +
      "If no network connects, the hotspot opens again.";

    function stopHotspot(): void {
      void wifiAction(
        "/api/wifi/hotspot",
        { action: "stop" },
        `${LOSE_CONNECTION}\n\nTurn off the hotspot?`,
      );
    }

    function startHotspot(): void {
      void wifiAction(
        "/api/wifi/hotspot",
        { action: "start" },
        `The windbot leaves ${wifi.value?.ssid ?? "the current network"} and opens the hotspot ` +
          `"${wifi.value?.hotspot_ssid}". Continue?`,
      );
    }

    function connectTo(network: KnownNetwork): void {
      void wifiAction(
        "/api/wifi/connect",
        { uuid: network.uuid },
        `${LOSE_CONNECTION}\n\nConnect to ${network.ssid}?`,
      );
    }

    onMounted(() => void poll());
    onUnmounted(() => {
      stopped = true;
      window.clearTimeout(timer);
    });

    return {
      wifi,
      wifiError,
      scanned,
      scanning,
      scanError,
      scan,
      busy,
      stopHotspot,
      startHotspot,
      connectTo,
      formatAge,
    };
  },
});
