<script lang="ts" setup>
import { ref } from "vue";
import { JoyType, joyInfo } from "../types/joy";
import { createTopic, useSubscriber } from "../script/rosHook";

const joyTopic = createTopic<JoyType>(joyInfo);
const joyData = useSubscriber(joyTopic);
</script>

<template>
  <q-card>
    <q-card-section class="q-pa-sm">
      <div class="text-h7">Joy</div>
    </q-card-section>
    <q-card-section class="q-pa-sm">
      <p>axes: {{ joyData?.axes }}</p>
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
  width: 1.1rem;
  height: 1.1rem;
  border-radius: 50%;
  color: white;
  font-size: small;
}
</style>
