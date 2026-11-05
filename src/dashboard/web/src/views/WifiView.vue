<script lang="ts" src="./WifiView.ts"></script>

<template>
  <main class="page">
    <div class="grid">
      <section class="card">
        <h2>Wi-Fi</h2>
        <div v-if="!wifi" class="muted">{{ wifiError ?? "Loading…" }}</div>
        <template v-else>
          <template v-if="wifi.mode === 'client'">
            <div>
              <strong>Connected to {{ wifi.ssid }}</strong>
              <template v-if="wifi.signal != null"> · signal {{ wifi.signal }}%</template>
            </div>
            <div class="muted">Address {{ wifi.address ?? "—" }}</div>
          </template>
          <template v-else-if="wifi.mode === 'hotspot'">
            <div>
              <strong>Hotspot "{{ wifi.hotspot_ssid }}" is open</strong>
              <template v-if="wifi.clients != null"> · {{ wifi.clients }} device(s)</template>
            </div>
            <div class="muted">Dashboard at http://{{ wifi.hotspot_address }}</div>
          </template>
          <div v-else-if="wifi.mode === 'searching'">
            <strong>{{ wifi.ssid ? `Connecting to ${wifi.ssid}…` : "Not connected" }}</strong>
          </div>
          <div v-else>The Wi-Fi interface isn't available.</div>
          <div v-if="wifi.action" class="banner">{{ wifi.action }}…</div>
          <div v-if="wifi.message" class="banner">{{ wifi.message }}</div>
          <div v-if="wifi.error" class="banner bad">{{ wifi.error }}</div>
          <div v-if="wifiError" class="banner bad">{{ wifiError }}</div>
          <div class="actions">
            <button v-if="wifi.mode === 'hotspot'" class="btn" :disabled="busy" @click="stopHotspot">
              Turn off hotspot and join a known network
            </button>
            <button
              v-if="wifi.mode !== 'hotspot' && wifi.mode !== 'unavailable'"
              class="btn"
              :disabled="busy"
              @click="startHotspot"
            >
              Open hotspot now
            </button>
          </div>
        </template>
      </section>

      <section class="card">
        <h2>Known networks</h2>
        <table>
          <thead>
            <tr><th>Network</th><th>Priority</th><th></th></tr>
          </thead>
          <tbody>
            <tr v-for="network in wifi?.networks ?? []" :key="network.uuid">
              <td>
                {{ network.ssid }}
                <span v-if="network.active" class="badge good">connected</span>
                <span v-if="!network.autoconnect" class="badge">manual</span>
              </td>
              <td>{{ network.priority }}</td>
              <td class="row-actions">
                <button
                  class="btn small"
                  :disabled="busy || network.active"
                  @click="connectTo(network)"
                >
                  Connect
                </button>
              </td>
            </tr>
            <tr v-if="wifi && wifi.networks.length === 0">
              <td colspan="3" class="muted">No known networks.</td>
            </tr>
          </tbody>
        </table>
        <p class="muted hint">The windbot joins the known network in reach with the highest priority.</p>
      </section>

      <section class="card">
        <h2>
          Networks in reach
          <button class="btn small scan" :disabled="scanning" @click="scan">Scan</button>
        </h2>
        <ul class="scan-list">
          <li v-if="scanning" class="muted">Scanning…</li>
          <li v-else-if="scanError" class="muted">{{ scanError }}</li>
          <li v-else-if="scanned === null" class="muted">Press Scan to look for networks.</li>
          <li v-else-if="scanned.length === 0" class="muted">No networks found.</li>
          <template v-else>
            <li v-for="network in scanned" :key="network.ssid">
              <span>
                {{ network.ssid }}
                <span v-if="network.in_use" class="badge good">connected</span>
              </span>
              <span class="muted">{{ network.security ? "🔒 " : "" }}{{ network.signal ?? "?" }}%</span>
            </li>
          </template>
        </ul>
      </section>
    </div>
  </main>
</template>

<style scoped>
.actions {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-top: 12px;
}
.row-actions {
  text-align: right;
}
.row-actions .btn {
  margin: 2px 0;
}
.hint {
  margin-top: 8px;
  font-size: 0.85rem;
}
.scan {
  margin-left: auto;
}
.scan-list {
  list-style: none;
  max-height: 260px;
  overflow-y: auto;
}
.scan-list li {
  display: flex;
  justify-content: space-between;
  gap: 8px;
  padding: 8px 4px;
  border-bottom: 1px solid #e4e6ec;
}
</style>
