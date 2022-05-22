<script lang="ts" setup>
import { ref } from "vue";
import { createTopic } from "../script/rosHook";

const lagoriSetTopic = createTopic<{
  data: number[];
}>({
  name: "/pile_status",
  messageType: "std_msgs::UInt8MultiArray",
});

type LagoriPostureType = "Vertical" | "Angled" | "Horizontal";
const initialPosture: { label: string; value: LagoriPostureType } = {
  label: "横",
  value: "Horizontal",
};
const lagoriPostures = ref<{ label: string; value: LagoriPostureType }[]>([
  initialPosture,
  initialPosture,
  initialPosture,
  initialPosture,
  initialPosture,
]);
const options: { label: string; value: LagoriPostureType }[] = [
  { label: "横", value: "Horizontal" },
  { label: "斜め", value: "Angled" },
  { label: "縦", value: "Vertical" },
];

const send = () => {
  const postures: Record<LagoriPostureType, number> = {
    Horizontal: 0,
    Angled: 1,
    Vertical: 2,
  };
  const laroriPostureNums = lagoriPostures.value.map((e) => postures[e.value]);

  lagoriSetTopic.publish({
    data: laroriPostureNums,
  });
};
</script>

<template>
  <q-page padding class="items-stretch column">
    <div
      class="col-10 row no-wrap justify-around items-start content-start q-px-lg"
    >
      <div
        v-for="(item, index) in lagoriPostures"
        :key="index"
        style="width: 5rem"
      >
        <div class="text-subtitle1">{{ index + 1 }}</div>
        <p>
          <q-select
            v-model="lagoriPostures[index]"
            :options="options"
            label="姿勢"
            style="animation-duration: 10ms"
          />
        </p>
      </div>
    </div>
    <div class="col">
      <q-space />
      <q-btn @click="send">SEND!</q-btn>
    </div>
  </q-page>
</template>

<style lang="scss"></style>
