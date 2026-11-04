<script lang="ts" src="./App.ts"></script>

<template>
  <h1>Windbot</h1>
  <p :class="{ warning: stale }">{{ state }}</p>
  <dl v-if="latest" :class="{ stale }">
    <template v-for="row in ROWS" :key="row.key">
      <dt>{{ row.label }}</dt>
      <dd>{{ format(row) }}</dd>
    </template>
  </dl>
</template>

<style>
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; margin: 24px; color: #1f2333; }
dl { display: grid; grid-template-columns: auto 1fr; gap: 6px 16px; max-width: 420px; }
dt { color: #6b6f80; }
dd { margin: 0; font-variant-numeric: tabular-nums; }
.stale { opacity: 0.4; }
.warning { color: #b3261e; font-weight: 600; }
</style>
