<script lang="ts" setup>
defineProps({
  title: String,
});
</script>

<template>
  <q-card>
    <q-card-section class="q-px-sm q-py-xs">{{ title }}</q-card-section>
    <q-card-section class="q-px-sm q-py-xs">
      <slot></slot>
    </q-card-section>
  </q-card>
</template>

<style lang="scss"></style>
