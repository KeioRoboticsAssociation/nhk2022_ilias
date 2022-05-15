<script lang="ts" setup>
import { computed } from "vue";
import { JoyType, joyInfo } from "../types/joy";
import { createTopic, useSubscriber } from "../script/rosHook";
import Card from "./Card.vue";

const joyTopic = createTopic<JoyType>(joyInfo);
const joyData = useSubscriber(joyTopic);

const axes = computed(() => joyData.value?.axes.map((e) => e.toFixed(3)));
</script>

<template>
  <Card :title="'Joy'">
    <p class="q-mb-sm">axes:</p>
    <p>R: {{ axes?.at(0) }}:{{ axes?.at(1) }}</p>
    <p>L: {{ axes?.at(2) }}:{{ axes?.at(3) }}</p>
    <div class="row wrap justify-start">
      <div
        v-for="(btn, index) in joyData?.buttons"
        :key="index"
        class="q-ma-xs"
      >
        <div v-if="btn" class="perfect-circle bg-positive">
          {{ index }}
        </div>
        <div v-else class="perfect-circle bg-red">
          {{ index }}
        </div>
      </div>
    </div>
  </Card>
</template>

<style lang="scss" scoped>
.perfect-circle {
  width: 1rem;
  height: 1rem;
  border-radius: 50%;
  color: white;
  font-size: 0.6rem;
  text-align: center;
}
</style>
