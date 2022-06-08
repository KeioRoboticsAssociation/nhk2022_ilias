<script lang="ts" setup>
import { ref } from "vue";
import { createTopic } from "../script/rosHook";

const lagoriSetTopic = createTopic<{
  layout: {
    dim: [];
    // eslint-disable-next-line @typescript-eslint/naming-convention
    data_offset: 0;
  };
  data: number[];
}>({
  name: "/pile_status",
  messageType: "std_msgs/UInt8MultiArray",
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
    layout: {
      dim: [],
      // eslint-disable-next-line @typescript-eslint/naming-convention
      data_offset: 0,
    },
    data: laroriPostureNums,
  });
};

const bohPosition = ref<number>(0);

const bohSend = (value: number | null) => {
  if (value == null) return;
  const bohTopic = createTopic<{
    id: number;
    data: number[];
  }>({
    name: "/send_serial",
    messageType: "rogi_link_msgs/RogiLink",
  });
  const buffer = Array.from(
    new Uint8Array(new Float32Array([value, 0]).buffer)
  );
  // eslint-disable-next-line no-bitwise
  const canId = (0x0c << 6) | 0x03;
  bohTopic.publish({
    id: canId,
    data: buffer,
  });
};
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col q-pa-sm">
      <p class="text-subtitle1">BoH位置</p>
      <div class="row justify-start q-mt-xl q-pl-xl" style="min-height: 50%">
        <q-slider
          v-model="bohPosition"
          :min="-6.85"
          :max="0"
          :step="0.05"
          thumb-size="70px"
          track-size="20px"
          vertical
          reverse
          label-always
          @update:model-value="bohSend"
        />
      </div>
    </div>
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

      <section class="row justify-end q-mt-sm">
        <q-btn color="primary" class="on-right" @click="send">
          ラゴリ状態セット
        </q-btn>
      </section>
    </div>
  </q-page>
</template>

<style lang="scss">
.q-slider--v {
  height: 300px !important;
}
</style>
