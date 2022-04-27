<script lang="ts" setup>
import { computed } from "vue";
import { JoyType, joyInfo } from "../types/joy";
import { createTopic, useSubscriber } from "../script/rosHook";

const joyTopic = createTopic<JoyType>(joyInfo);
const joyData = useSubscriber(joyTopic);

const axes = computed(() => joyData.value?.axes.map((e) => e.toFixed(3)));
</script>

<template>
  <q-card>
    <q-card-section class="q-px-sm q-py-xs">
      <div class="text-h7">Joy</div>
    </q-card-section>
    <q-card-section class="q-px-sm q-py-xs">
      <p class="q-mb-sm">axes: {{ axes }}</p>
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
    </q-card-section>
  </q-card>
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
