<template>
  <div id="app" class="dashboard">
    <header class="header">
      <button
        class="settings-btn"
        @click="
          pendingTimeFrame = timeFrameMinutes;
          pendingWindSpeedUnit = windSpeedUnit;
          pendingSensorSpeedThreshold = roundOne(
            convertWindSpeedFromMps(sensorSpeedThresholdMps, pendingWindSpeedUnit)
          );
          pendingOldestAtTop = directionOldestAtTop;
          pendingExclusionWindowSec = exclusionWindowSec;
          pendingHideInvalidData = hideInvalidData;
          pendingAccelThresholdMs2 = accelThresholdMs2;
          pendingHeadingRateThresholdDegs = headingRateThresholdDegs;
          pendingAvgWindowMinutes = avgWindowMinutes;
          pendingMinChartWidthDeg = minChartWidthDeg;
          pendingMaxDisplaySpanDeg = maxDisplaySpanDeg;
          pendingSmoothing = smoothingWindowSec;
          settingsOpen = true;
        "
        aria-label="Open settings"
      >
        ⚙
      </button>
      <div v-if="error" class="error-banner">{{ error }}</div>
      <div
        v-if="staleDataNotice"
        :class="['stale-banner', staleAlertLevel === 'error' ? 'stale-error' : 'stale-warning']"
      >
        {{ staleDataNotice }}
      </div>
    </header>

    <!-- Settings popup -->
    <div
      v-if="settingsOpen"
      class="settings-overlay"
      @click.self="settingsOpen = false"
    >
      <div class="settings-panel">
        <div class="settings-header">
          <h2>Settings</h2>
          <button
            class="settings-close"
            @click="settingsOpen = false"
            aria-label="Close settings"
          >
            ✕
          </button>
        </div>

        <div class="settings-row">
          <label for="timeframe-select">Time frame</label>
          <select id="timeframe-select" v-model.number="pendingTimeFrame">
            <option
              v-for="m in [5, 10, 15, 30, 45, 60, 75, 90]"
              :key="m"
              :value="m"
            >
              {{ m }} min
            </option>
          </select>
        </div>

        <div class="settings-row">
          <label for="speed-unit-select">Wind speed unit</label>
          <select id="speed-unit-select" v-model="pendingWindSpeedUnit">
            <option value="mps">Meters per second (m/s)</option>
            <option value="knots">Knots (kn)</option>
            <option value="kmh">Kilometres per hour (km/h)</option>
          </select>
        </div>

        <div class="settings-row">
          <label for="dir-order-toggle">Oldest at top</label>
          <input
            id="dir-order-toggle"
            type="checkbox"
            v-model="pendingOldestAtTop"
          />
        </div>

        <div class="settings-row">
          <label for="exclusion-window-select">Exclusion window</label>
          <select
            id="exclusion-window-select"
            v-model.number="pendingExclusionWindowSec"
          >
            <option :value="0">Off</option>
            <option v-for="s in [2, 5, 10, 15, 20, 30]" :key="s" :value="s">
              {{ s }} s
            </option>
          </select>
        </div>

        <div class="settings-row">
          <label for="hide-invalid-toggle">Hide invalid data</label>
          <input
            id="hide-invalid-toggle"
            type="checkbox"
            v-model="pendingHideInvalidData"
          />
        </div>

        <div class="settings-row">
          <label for="accel-threshold-input">Accel threshold (m/s²)</label>
          <input
            id="accel-threshold-input"
            type="number"
            min="0"
            step="0.05"
            v-model.number="pendingAccelThresholdMs2"
          />
        </div>

        <div class="settings-row">
          <label for="heading-threshold-input"
            >Heading rate threshold (°/s)</label
          >
          <input
            id="heading-threshold-input"
            type="number"
            min="0"
            step="0.5"
            v-model.number="pendingHeadingRateThresholdDegs"
          />
        </div>

        <div class="settings-row">
          <label for="sensor-speed-threshold-input"
            >Sensor Speed Threshold ({{ pendingWindSpeedUnitLabel }})</label
          >
          <input
            id="sensor-speed-threshold-input"
            type="number"
            min="0"
            step="0.1"
            v-model.number="pendingSensorSpeedThreshold"
          />
        </div>

        <div class="settings-row">
          <label for="avg-window-select">Average window</label>
          <select
            id="avg-window-select"
            v-model.number="pendingAvgWindowMinutes"
          >
            <option
              v-for="m in [5, 10, 15, 30, 45, 60]"
              :key="m"
              :value="m"
            >
              {{ m }} min
            </option>
          </select>
        </div>

        <div class="settings-row">
          <label for="min-chart-width-input">Min chart width (deg)</label>
          <input
            id="min-chart-width-input"
            type="number"
            min="10"
            max="360"
            v-model.number="pendingMinChartWidthDeg"
          />
        </div>

        <div class="settings-row">
          <label for="max-display-span-select">Max display span</label>
          <select
            id="max-display-span-select"
            v-model.number="pendingMaxDisplaySpanDeg"
          >
            <option v-for="d in [45, 90, 180, 270, 360]" :key="d" :value="d">
              {{ d }}°{{ d === 360 ? " (default)" : "" }}
            </option>
          </select>
        </div>

        <div class="settings-row">
          <label for="smoothing-select">Chart smoothing</label>
          <select id="smoothing-select" v-model.number="pendingSmoothing">
            <option :value="5">5 s (none)</option>
            <option v-for="s in [10, 15, 30, 45, 60, 90, 120]" :key="s" :value="s">
              {{ s }} s
            </option>
          </select>
        </div>

        <div class="settings-footer">
          <button class="apply-btn" @click="applySettings">Apply</button>
        </div>
      </div>
    </div>

    <main v-if="!loading" class="content">
      <div class="charts">
        <div class="left-column">
          <div class="kpi-cards" :class="kpiValueAgeClass">
            <div class="kpi-card">
              <div class="label">True Wind Speed</div>
              <div class="value">
                {{
                  latestDisplayMeasurement
                    ? convertWindSpeed(latestDisplayMeasurement.true_wind_speed_mps).toFixed(
                        1
                      )
                    : "—"
                }}
                {{ windSpeedUnitLabel }}
              </div>
            </div>
            <div class="kpi-card">
              <div class="label">True Wind Direction</div>
              <div class="value">
                {{ latestDisplayMeasurement?.true_wind_dir_deg?.toFixed(0) || "—" }}°
              </div>
            </div>
            <div class="kpi-card">
              <div class="label">Avg True Wind Speed ({{ avgWindow }})</div>
              <div class="value">
                {{
                  averageMeasurement
                    ? convertWindSpeed(
                        averageMeasurement.true_wind_speed_mps
                      ).toFixed(1)
                    : "—"
                }}
                {{ windSpeedUnitLabel }}
              </div>
            </div>
            <div class="kpi-card">
              <div class="label">Avg True Wind Direction ({{ avgWindow }})</div>
              <div class="value">{{ averageMeasurement?.true_wind_dir_deg?.toFixed(0) || "—" }}°</div>
            </div>
          </div>
          <div id="wind-speed-chart" class="chart-container"></div>
        </div>
        <div id="wind-direction-chart" class="chart-container"></div>
      </div>
    </main>

    <div v-else class="loading">
      <p>Loading wind data...</p>
    </div>
  </div>
</template>

<script lang="ts" src="./App.ts"></script>

<style>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}
</style>

<style scoped>
#app {
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen,
    Ubuntu, Cantarell, sans-serif;
  min-height: 100vh;
  padding: 20px;
  display: flex;
  flex-direction: column;
}

.header {
  position: relative;
}

.header h1 {
  font-size: 3rem;
  margin-bottom: 10px;
}

.error-banner {
  background: rgba(255, 0, 0, 0.8);
  color: white;
  padding: 10px 20px;
  border-radius: 8px;
  margin-top: 10px;
}

.stale-banner {
  padding: 13px 20px;
  border-radius: 8px;
  margin-bottom: 16px;
  font-weight: 600;
}

.stale-banner.stale-warning {
  background: rgba(255, 165, 0, 0.18);
  color: #8a4b00;
}

.stale-banner.stale-error {
  background: rgba(220, 38, 38, 0.2);
  color: #991b1b;
}

.content {
  animation: fadeIn 0.5s ease-in-out;
  display: flex;
  flex-direction: column;
  gap: 40px;
  flex-grow: 1;
}

.kpi-cards {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 20px;
}

.kpi-card {
  background: rgba(255, 255, 255, 0.95);
  padding: 25px;
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  backdrop-filter: blur(4px);
  transition: transform 0.3s ease, box-shadow 0.3s ease;
}

.kpi-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 12px 40px rgba(0, 0, 0, 0.15);
}

.kpi-card .label {
  font-size: 0.9rem;
  color: #666;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  margin-bottom: 8px;
  font-weight: 600;
}

.kpi-card .value {
  font-size: 2rem;
  color: #333;
  font-weight: bold;
}

.kpi-cards.kpi-stale-warning .kpi-card .value {
  color: #b45309;
}

.kpi-cards.kpi-stale-error .kpi-card .value {
  color: #b91c1c;
}

.charts {
  display: grid;
  flex-grow: 1;
  grid-template-columns: repeat(auto-fit, minmax(500px, 1fr));
  gap: 20px;
}

.left-column {
  display: flex;
  flex-direction: column;
  gap: 20px;
}


.chart-container {
  background: rgba(255, 255, 255, 0.95);
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1);
  min-height: 500px;
  padding: 20px;
  flex-grow: 1;
}

.loading {
  text-align: center;
  color: white;
  font-size: 1.2rem;
  padding: 60px 20px;
}

/* ── Settings ─────────────────────────────────────────── */
.settings-btn {
  position: absolute;
  top: 0px;
  right: 0px;
  z-index: 1;
  background: rgba(255, 255, 255, 0.2);
  border: none;
  border-radius: 50%;
  width: 44px;
  height: 44px;
  font-size: 44px;
  cursor: pointer;
  color: black;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: background 0.2s;
}

.settings-btn:hover {
  background: rgba(255, 255, 255, 0.35);
}

.settings-overlay {
  position: fixed;
  inset: 0;
  background: rgba(0, 0, 0, 0.45);
  z-index: 100;
  display: flex;
  align-items: center;
  justify-content: center;
}

.settings-panel {
  background: #fff;
  border-radius: 14px;
  padding: 28px 32px;
  min-width: 320px;
  box-shadow: 0 16px 48px rgba(0, 0, 0, 0.2);
}

.settings-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 24px;
}

.settings-header h2 {
  font-size: 1.3rem;
  color: #333;
  margin: 0;
}

.settings-close {
  background: none;
  border: none;
  font-size: 1.2rem;
  cursor: pointer;
  color: #666;
  line-height: 1;
}

.settings-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 18px;
  gap: 16px;
}

.settings-row label {
  font-size: 0.95rem;
  color: #444;
  font-weight: 500;
}

.settings-row select {
  padding: 6px 10px;
  border-radius: 8px;
  border: 1px solid #ccc;
  font-size: 0.95rem;
  cursor: pointer;
}

.settings-row input[type="checkbox"] {
  width: 18px;
  height: 18px;
  cursor: pointer;
}

.settings-footer {
  display: flex;
  justify-content: flex-end;
  margin-top: 8px;
}

.apply-btn {
  background: #667eea;
  color: white;
  border: none;
  border-radius: 8px;
  padding: 8px 22px;
  font-size: 0.95rem;
  font-weight: 600;
  cursor: pointer;
  transition: opacity 0.2s;
}

.apply-btn:hover {
  opacity: 0.88;
}

/* ── Animations ───────────────────────────────────────── */
@keyframes fadeIn {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

@media (max-width: 768px) {
  .header h1 {
    font-size: 1.8rem;
  }

  .kpi-cards {
    grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
    gap: 15px;
  }

  .kpi-card .value {
    font-size: 1.5rem;
  }

  .charts {
    grid-template-columns: 1fr;
  }

  .chart-container {
    height: 400px;
  }
}
</style>
