<script lang="ts" setup>
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
  cancel.value();
};
</script>

<template>
  <Card title="AutoPath">
    <q-btn @click="() => sendGoal({ pathmode: 1, direction: 1 })">send!</q-btn>
    <p>{{ feedback }}</p>
    <p>{{ result }}</p>
    <p>{{ status }}</p>
    <q-btn @click="() => cancelAction()">cancel</q-btn>
  </Card>
</template>
