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
  const {x,y,z,w} = odomData.value.pose.pose.orientation;
  return qte([w, x, y, z])[2];
});
</script>

<template>
  <Card title="Odom">
    <div class="text-subtitle2">Pose</div>
    <p>
      X: {{ position?.x.toFixed(3) }}, Y: {{ position?.y.toFixed(3) }}, θ:
      {{ theta.toFixed(3) }}
    </p>
    <div class="text-subtitle2">Velocity</div>
    <p>
      X: {{ velocity?.x.toFixed(3) }}, Y: {{ velocity?.y.toFixed(3) }}, ω:
      {{ omega?.toFixed(3) }}
    </p>
  </Card>
</template>

<style lang="scss"></style>
