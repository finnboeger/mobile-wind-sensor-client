import vue from "@vitejs/plugin-vue";
import { defineConfig } from "vite";

// `npm run dev` proxies the API to a running windbot, e.g.
// WINDBOT_URL=http://10.42.0.1 npm run dev
const windbot = process.env.WINDBOT_URL ?? "http://localhost:80";

export default defineConfig({
  plugins: [vue()],
  build: {
    // served by src/dashboard/server.py, gitignored and shipped in the releases
    outDir: "../static",
    emptyOutDir: true,
    // ECharts alone is about 500 kB, served gzipped by the windbot
    chunkSizeWarningLimit: 800,
  },
  server: {
    proxy: { "/api": windbot },
  },
});
