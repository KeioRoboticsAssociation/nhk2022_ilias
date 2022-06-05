<script lang="ts" setup>
import { ref } from "vue";

import { createTopic } from "../script/rosHook";

import Card from "./Card.vue";

const lagoriNumberTopic = createTopic<{
  data: number;
}>({
  name: "/lagori_number",
  messageType: "std_msgs::UInt8",
});

const lagoriSize = ref<0 | 1 | 2 | 3 | 4 | 5>(5);

const setArm = () => {
  console.log("publish");
  lagoriNumberTopic.publish({
    data: lagoriSize.value,
  });
};
</script>

<template>
  <Card title="アーム選択">
    <div>
      <div class="text-subtitle1">ラゴリサイズ: {{ lagoriSize }}</div>
      <q-btn-toggle
        v-model="lagoriSize"
        toggle-color="primary"
        :options="[
          { label: '1', value: 1 },
          { label: '2', value: 2 },
          { label: '3', value: 3 },
          { label: '4', value: 4 },
          { label: '5', value: 5 },
        ]"
        size="lg"
        style="text-align: center"
        @update:model-value="setArm"
      ></q-btn-toggle>
      <q-btn
        @click="
          () => {
            lagoriSize = 0;
            setArm();
          }
        "
      >
        Reset
      </q-btn>
    </div>
  </Card>
</template>

<style lang="scss"></style>
