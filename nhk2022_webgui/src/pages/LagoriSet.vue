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
const lagoriPostures = ref<LagoriPostureType[]>([
  "Horizontal",
  "Horizontal",
  "Horizontal",
  "Horizontal",
  "Horizontal",
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
  const laroriPostureNums = lagoriPostures.value.map((e) => postures[e]);

  lagoriSetTopic.publish({
    data: laroriPostureNums,
  });
};
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-8">
      <q-list separator>
        <q-item
          v-for="(item, index) in lagoriPostures"
          :key="index"
          class="q-pa-xs"
        >
          <q-item-section>
            <span class="col-2 text-subtitle2">
              {{ index + 1 }}
            </span>
          </q-item-section>
          <q-item-section side>
            <q-btn-toggle
              v-model="lagoriPostures[index]"
              toggle-color="primary"
              text-color="grey-9"
              :options="options"
              class="col"
            />
          </q-item-section>
        </q-item>
      </q-list>

      <q-btn color="primary" @click="send">ラゴリ状態セット</q-btn>
    </div>
    <div class="col">
      <q-space />
    </div>
  </q-page>
</template>

<style lang="scss"></style>
