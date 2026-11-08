<script lang="ts" src="./LiveView.ts"></script>

<template>
  <main class="page">
    <div v-if="unreachable" class="banner bad">The windbot can't be reached.</div>
    <p v-else-if="!status" class="muted">Loading…</p>
    <div class="grid">
      <section class="card">
        <h2>
          Internet check
          <span class="age">{{ checking ? "checking…" : checkedAgo }}</span>
          <button class="btn small" :disabled="checking" @click="checkInternet">Check now</button>
        </h2>
        <p v-if="internetError" class="muted">The check failed: {{ internetError }}</p>
        <dl v-if="internet">
          <template v-for="row in internetRows" :key="row.label">
            <dt>{{ row.label }}</dt>
            <dd :class="row.level">{{ row.text }}</dd>
            <dd v-if="row.detail" class="detail">{{ row.detail }}</dd>
          </template>
        </dl>
        <p v-else-if="!internetError" class="muted">Checking…</p>
      </section>
      <section v-for="card in cards" :key="card.title" class="card">
        <h2>
          {{ card.title }}
          <span v-if="card.section !== undefined" :class="['age', age(card.section).level]">
            {{ age(card.section).text }}
          </span>
        </h2>
        <dl>
          <template v-for="[key, value] in card.rows" :key="key">
            <dt>{{ key }}</dt>
            <dd>{{ value }}</dd>
          </template>
        </dl>
      </section>
    </div>
  </main>
</template>

<style scoped>
.age {
  margin-left: auto;
  font-size: 0.75rem;
  font-weight: 400;
  color: #6b6f80;
}
.age.warn {
  color: #b45309;
}
.age.bad {
  color: #c62828;
}
dl {
  display: grid;
  grid-template-columns: auto 1fr;
  gap: 4px 12px;
  font-size: 0.9rem;
}
dt {
  color: #6b6f80;
}
dd {
  font-variant-numeric: tabular-nums;
  text-align: right;
}
dd.good {
  color: #2f855a;
}
dd.bad {
  color: #c62828;
}
dd.detail {
  grid-column: 1 / -1;
  font-size: 0.8rem;
  color: #6b6f80;
  overflow-wrap: anywhere;
}
</style>
