<script lang="ts" src="./LiveView.ts"></script>

<template>
  <main class="page">
    <div v-if="unreachable" class="banner bad">The windbot can't be reached.</div>
    <p v-else-if="!status" class="muted">Loading…</p>
    <div class="grid">
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
</style>
