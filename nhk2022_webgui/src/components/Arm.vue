<script lang="ts" setup>
import { ref } from "vue";

import { createTopic } from "../script/rosHook";

import Card from "./Card.vue";

const elevatorTopic = createTopic<{
  data: number;
}>({
  name: "/auto_elev",
  messageType: "std_msgs::Float32",
});
const grabTopic = createTopic<{
  data: number;
}>({
  name: "/auto_grab",
  messageType: "std_msgs::Float32",
});

const ragoriSize = ref<1 | 2 | 3 | 4 | 5>(5);
const ragoriVH = ref<"vertical" | "horizontal">("vertical");

const setArm = () => {
  const grabTable = [20, 30, 40, 50, 60];
  const elevatorVerticalTable = [2, 3, 4, 5, 6];
  const elevatorHorizontal = 1;

  grabTopic.publish({ data: grabTable[ragoriSize.value] });

  const elevatorPosition =
    ragoriVH.value === "horizontal"
      ? elevatorHorizontal
      : elevatorVerticalTable[ragoriSize.value];
  elevatorTopic.publish({ data: elevatorPosition });
};
</script>

<template>
  <Card title="アーム選択">
    <div>
      <div class="text-subtitle1">ラゴリサイズ: {{ ragoriSize }}</div>
      <q-btn-toggle
        v-model="ragoriSize"
        toggle-color="primary"
        :options="[
          { label: '1', value: 1 },
          { label: '2', value: 2 },
          { label: '3', value: 3 },
          { label: '4', value: 4 },
          { label: '5', value: 5 },
        ]"
      ></q-btn-toggle>
    </div>

    <div class="q-my-sm">
      <div class="text-subtitle1">向き:</div>
      <q-btn-toggle
        v-model="ragoriVH"
        toggle-color="primary"
        :options="[
          { label: '縦', value: 'vertical' },
          { label: '横', value: 'horizontal' },
        ]"
      />
    </div>

    <div class="q-mt-lg">
      <q-btn style="width: 100%" @click="setArm">send!</q-btn>
    </div>
  </Card>
</template>

<style lang="scss"></style>
