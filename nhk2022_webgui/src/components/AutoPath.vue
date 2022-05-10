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

const pathAction = createActionClient(pursuitPathActionInfo, 100);
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

const goalNum = ref<number>(0);
const dirNum = ref<number>(0);
const send = () => {
  sendGoal({
    pathmode: goalNum.value,
    direction: dirNum.value,
  });
};
</script>

<template>
  <Card title="AutoPath">
    <p>
      path:
      <q-slider v-model="goalNum" :min="1" :max="5" label label-always />
    </p>
    <p>
      dir:
      <q-slider v-model="dirNum" :min="0" :max="1" label label-always />
    </p>
    <q-btn @click="send">send!</q-btn>
    <p>{{ feedback }}</p>
    <p>{{ result }}</p>
    <p>{{ status }}</p>
    <q-btn @click="() => cancelAction()">cancel</q-btn>
  </Card>
</template>
