<script lang="ts" setup>
defineProps<{
  title: string;
}>();
</script>

<template>
  <q-card>
    <q-card-section class="q-px-sm q-py-xs">
      <div class="text-subtitle1">{{ title }}</div>
    </q-card-section>
    <q-separator />
    <q-card-section class="q-px-sm q-py-xs">
      <slot></slot>
    </q-card-section>
  </q-card>
</template>

<style lang="scss"></style>
