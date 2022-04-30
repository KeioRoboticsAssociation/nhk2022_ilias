<script lang="ts" setup>
import qte from "quaternion-to-euler";
import { computed } from "vue";
import { OdomType, odomInfo } from "../types/odom";
import { createTopic, useSubscriber } from "../script/rosHook";
import Card from "./Card.vue";

const odomTopic = createTopic<OdomType>(odomInfo);
const odomData = useSubscriber(odomTopic);
const position = computed(() => odomData.value?.pose.pose.position);
const velocity = computed(() => odomData.value?.twist.twist.liniar);
const omega = computed(() => odomData.value?.twist.twist.angular.z);
const theta = computed(() => {
  if (odomData.value?.pose.pose.orientation == null) {
    return 0;
  }
  return qte(Object.values(odomData.value.pose.pose.orientation))[2];
});
</script>

<template>
  <Card title="Odom">
    <p>Pose: X: {{ position?.x }}, Y: {{ position?.y }}, θ: {{ theta }}</p>
    <p>Velocity: X: {{ velocity?.x }}, Y: {{ velocity?.y }}, ω: {{ omega }}</p>
  </Card>
</template>

<style lang="scss"></style>
