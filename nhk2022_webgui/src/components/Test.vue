<script lang="ts" setup>
import { computed } from "vue";
import { CmdVelPacket, cmdVelInfo } from "../types/cmdVel";
import {
  createTopic,
  useSubscriber,
  publishTopic,
  useRosStatus,
  connectRos,
} from "../script/rosHook";

const { isConnected } = useRosStatus();

const cmdVelTopic = computed(() => createTopic<CmdVelPacket>(cmdVelInfo));
const velData = useSubscriber(cmdVelTopic.value);

const cmdX = computed(() => velData.value?.linear.x);
const cmdY = computed(() => velData.value?.linear.y);
</script>

<template>
  <div>
    <p>isConnected: {{ isConnected }}</p>
    <p>cmdX: {{ cmdX }}</p>
    <p>cmdY: {{ cmdY }}</p>
  </div>
</template>

<style lang="scss" scoped></style>
