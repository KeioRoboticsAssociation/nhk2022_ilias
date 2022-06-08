<script lang="ts" setup>
import { ref } from "vue";
import { useRouter } from "vue-router";
import { useRosStatus, connectRos, createTopic } from "./script/rosHook";

const router = useRouter();

const { isConnected } = useRosStatus();

const url = ref<string>(`ws://${window.location.hostname}:9090`);

const emergencyStopTopic = createTopic<Record<string, never>>({
  name: "/emergency_stop_flag",
  messageType: "std_msgs::Empty",
});
const sendEmergency = () => {
  emergencyStopTopic.publish({});
};
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

        <q-chip
          color="red"
          text-color="white"
          icon="stop_circle"
          clickable
          @click="sendEmergency"
        >
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
@font-face {
  font-family: "BIZ UDPGothic"; /* フォント名 */
  /* フォーマットごとにパスを指定 */
  src: url("./assets/BIZUDPGothic-Regular.ttf") format("truetype");
}
#app {
  font-family: "BIZ UDPGothic", sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;

  font-size: 12px;
}
* {
  touch-action: manipulation;
}
</style>
