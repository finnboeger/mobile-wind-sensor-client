import {
  computed,
  defineComponent,
  nextTick,
  onMounted,
  onUnmounted,
  reactive,
  ref,
} from "vue";
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

    function deleteNetwork(network: KnownNetwork): void {
      const warning = network.active
        ? `The windbot is connected to ${network.ssid} and leaves it right away. If this ` +
          "page is reached through that network, it loses its connection. If no other " +
          "network connects, the hotspot opens after the fallback delay.\n\n" +
          `Forget ${network.ssid}?`
        : `Forget ${network.ssid}?`;
      void wifiAction("/api/wifi/networks/delete", { uuid: network.uuid }, warning);
    }

    // ---------------------------------------------------------------- network form

    const emptyForm = () => ({
      /** uuid of the known network being edited, null to add one */
      uuid: null as string | null,
      ssid: "",
      password: "",
      open: false,
      priority: 0,
      autoconnect: true,
      hidden: false,
    });
    const form = reactive(emptyForm());
    const ssidInput = ref<HTMLInputElement | null>(null);
    const passwordInput = ref<HTMLInputElement | null>(null);

    function resetForm(): void {
      Object.assign(form, emptyForm());
    }

    function editNetwork(network: KnownNetwork): void {
      Object.assign(form, emptyForm(), {
        uuid: network.uuid,
        ssid: network.ssid,
        priority: network.priority,
        autoconnect: network.autoconnect,
      });
      void nextTick(() => ssidInput.value?.focus());
    }

    function pickScanned(network: ScannedNetwork): void {
      Object.assign(form, emptyForm(), { ssid: network.ssid, open: !network.security });
      void nextTick(() => passwordInput.value?.focus());
    }

    async function saveNetwork(): Promise<void> {
      try {
        await postJson("/api/wifi/networks", {
          uuid: form.uuid,
          ssid: form.ssid,
          open: form.open,
          // an empty field keeps the current password of a known network
          password: form.open || !form.password ? null : form.password,
          priority: Number(form.priority) || 0,
          autoconnect: form.autoconnect,
          hidden: form.hidden,
        });
        wifiError.value = null;
        resetForm();
      } catch (e) {
        wifiError.value = e instanceof Error ? e.message : String(e);
      }
      await poll();
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
      deleteNetwork,
      form,
      ssidInput,
      passwordInput,
      resetForm,
      editNetwork,
      pickScanned,
      saveNetwork,
      formatAge,
    };
  },
});
