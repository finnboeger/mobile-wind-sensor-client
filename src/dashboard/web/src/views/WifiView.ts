import { defineComponent, onMounted, onUnmounted, ref } from "vue";
import { fetchWifi, scanWifi, type ScannedNetwork, type WifiStatus } from "../api";

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

    // One request at a time, like on the other tabs; a stalled one times out.
    async function poll(): Promise<void> {
      try {
        wifi.value = await fetchWifi();
        wifiError.value = null;
      } catch (e) {
        wifiError.value = e instanceof Error ? e.message : String(e);
      }
      if (!stopped) timer = window.setTimeout(poll, POLL_INTERVAL_MS);
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

    onMounted(() => void poll());
    onUnmounted(() => {
      stopped = true;
      window.clearTimeout(timer);
    });

    return { wifi, wifiError, scanned, scanning, scanError, scan };
  },
});
