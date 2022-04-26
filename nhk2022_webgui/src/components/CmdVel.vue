<script lang="ts" setup>
import { computed } from "vue";
import { CmdVelPacket, cmdVelInfo } from "../types/cmdVel";
import { createTopic, useSubscriber } from "../script/rosHook";

const cmdVelTopic = createTopic<CmdVelPacket>(cmdVelInfo);
const cmdVelData = useSubscriber(cmdVelTopic);
const linearX = computed(() => cmdVelData.value?.linear.x);
const linearY = computed(() => cmdVelData.value?.linear.y);
const angularZ = computed(() => cmdVelData.value?.angular.z);
</script>

<template>
  <q-card class="card">
    <q-card-section class="q-pa-sm">
      <div class="text-h7">CmdVel</div>
    </q-card-section>
    <q-card-section class="q-pa-sm">
      <p>x: {{ linearX }}</p>
      <p>y: {{ linearY }}</p>
      <p>θ: {{ angularZ }}</p>
    </q-card-section>
  </q-card>
</template>

<style lang="scss" scoped>
.card {
  height: 100%;
}
</style>
