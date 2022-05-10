<script lang="ts" setup>
import { computed } from "vue";
import { CmdVelPacket, cmdVelInfo } from "../types/cmdVel";
import { createTopic, useSubscriber } from "../script/rosHook";
import Card from "./Card.vue";

const cmdVelTopic = createTopic<CmdVelPacket>(cmdVelInfo);
const cmdVelData = useSubscriber(cmdVelTopic);
const linearX = computed(() => cmdVelData.value?.linear.x);
const linearY = computed(() => cmdVelData.value?.linear.y);
const angularZ = computed(() => cmdVelData.value?.angular.z);
</script>

<template>
  <Card title="CmdVel">
    <p>x: {{ linearX }}</p>
    <p>y: {{ linearY }}</p>
    <p>ω: {{ angularZ }}</p>
  </Card>
</template>

<style lang="scss" scoped></style>
