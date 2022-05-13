<script lang="ts" setup>
import { ref } from "vue";
import {
  pursuitPathActionInfo,
  PursuitPathFeedback,
  PursuitPathGoal,
  PursuitPathResult,
} from "../types/pursuitPath";
import { createActionClient, useGoal } from "../script/rosHook";

import Card from "./Card.vue";

const pathAction = createActionClient(pursuitPathActionInfo, 10000);
const { feedback, result, cancel, status, sendGoal } = useGoal<
  PursuitPathGoal,
  PursuitPathFeedback,
  PursuitPathResult
>(pathAction);
const cancelAction = () => {
  if (cancel.value === undefined) return;
  console.log("cancel");
  cancel.value();
};

const goalNum = ref<number>(1);
const dirNum = ref<number>(0);

const opensDialog = ref<boolean>(false);
const send = () => {
  sendGoal({
    pathmode: goalNum.value,
    direction: dirNum.value,
  });
  opensDialog.value = false;
};
</script>

<template>
  <Card title="AutoPath">
    <q-dialog v-model="opensDialog">
      <q-card style="width: 80vw">
        <q-card-section>
          <div class="text-h6">パス選択</div>
        </q-card-section>

        <q-card-section class="q-pt-none">
          <p>
            path:{{ goalNum }}
            <q-slider v-model="goalNum" :min="1" :max="5" label markers />
          </p>
          <p>Dir:</p>
          <q-btn-toggle
            v-model="dirNum"
            toggle-color="primary"
            :options="[
              { label: '0', value: 0 },
              { label: '1', value: 1 },
            ]"
          />
        </q-card-section>

        <q-card-actions align="right">
          <q-btn flat @click="send">send!</q-btn>
        </q-card-actions>
      </q-card>
    </q-dialog>
    <q-btn
      @click="
        () => {
          opensDialog = true;
        }
      "
    >
      パス設定
    </q-btn>
    <p>{{ feedback }}</p>
    <p>{{ result }}</p>
    <p>{{ status }}</p>
    <q-btn @click="() => cancelAction()">cancel</q-btn>
  </Card>
</template>
