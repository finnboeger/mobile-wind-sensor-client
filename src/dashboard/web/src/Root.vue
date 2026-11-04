<script lang="ts" src="./Root.ts"></script>

<template>
  <nav class="tabs">
    <button :class="{ active: tab === 'wind' }" @click="tab = 'wind'">Wind</button>
    <button :class="{ active: tab === 'live' }" @click="tab = 'live'">Live</button>
  </nav>
  <!-- kept alive so the history isn't loaded from the windbot again -->
  <WindView v-show="tab === 'wind'" />
  <LiveView v-if="tab === 'live'" />
</template>

<style>
body {
  background: #f4f5f7;
  color: #1f2333;
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Oxygen, Ubuntu,
    Cantarell, sans-serif;
}
</style>

<style scoped>
.tabs {
  display: flex;
  gap: 4px;
  padding: 12px 20px 0;
}
.tabs button {
  border: 1px solid #d9dbe3;
  background: #fff;
  color: inherit;
  border-radius: 8px;
  padding: 8px 14px;
  font-size: 0.95rem;
  cursor: pointer;
}
.tabs button.active {
  background: #5470c6;
  border-color: #5470c6;
  color: #fff;
}
</style>
