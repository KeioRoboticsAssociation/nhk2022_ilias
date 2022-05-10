<script lang="ts" setup>
import { ref } from "vue";
import { useRouter } from "vue-router";
import { CmdVelPacket, cmdVelInfo } from "./types/cmdVel";
import {
  createTopic,
  publishTopic,
  useRosStatus,
  connectRos,
} from "./script/rosHook";

const router = useRouter();

const { isConnected } = useRosStatus();

const url = ref<string>("ws://localhost:9090");

function send() {
  console.log("send");
  const msgTopic = createTopic<CmdVelPacket>(cmdVelInfo);
  publishTopic(msgTopic, {
    linear: {
      x: 5,
      y: 0,
      z: 0,
    },
    angular: {
      x: 5,
      y: 0,
      z: 0,
    },
  });
}
</script>

<template>
  <q-layout view="hHh lpR fFf" class="fullscreen">
    <q-header elevated class="bg-primary text-white">
      <q-toolbar>
        <q-toolbar-title>
          <q-btn
            flat
            round
            dense
            icon="arrow_back"
            @click="() => router.push('/')"
          />
          {{ $route.name }}
        </q-toolbar-title>
        <q-space />
        <q-chip v-if="isConnected" color="secondary" text-color="white">
          Connected
        </q-chip>
        <q-chip v-else color="red" text-color="white">Disconnected</q-chip>

        <q-chip color="red" text-color="white" icon="stop_circle" clickable>
          STOP
        </q-chip>
      </q-toolbar>
    </q-header>

    <q-page-container>
      <q-page v-if="!isConnected" class="q-pa-md">
        <q-input v-model="url" label="ROS Bridge URL" />
        <q-btn @click="() => connectRos(url)">connect!</q-btn>
      </q-page>
      <router-view v-else />
    </q-page-container>
  </q-layout>
</template>

<style lang="scss">
@import url("https://fonts.googleapis.com/css2?family=BIZ+UDPGothic&display=swap");
#app {
  font-family: "BIZ UDPGothic", sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;

  font-size: 12px;
}
</style>
