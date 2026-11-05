import { defineComponent, ref, watch } from "vue";
import LiveView from "./views/LiveView.vue";
import WindView from "./views/WindView.vue";

export default defineComponent({
  components: { LiveView, WindView },
  setup() {
    type Tab = "wind" | "live";
    const tab = ref<Tab>("wind");

    // The charts were laid out while hidden, so they need to measure themselves again
    watch(tab, (value) => {
      if (value === "wind") requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
    });

    return { tab };
  },
});
