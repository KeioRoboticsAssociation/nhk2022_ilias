<script lang="ts" setup>
import { computed } from "vue";
import { CmdVelPacket, cmdVelInfo } from "./types/cmdVel";
import {
  createTopic,
  useSubscriber,
  publishTopic,
  useRosStatus,
  connectRos,
} from "./script/rosHook";
import Test from "./components/Test.vue";

const { isConnected } = useRosStatus();

const cmdVelTopic = computed(() => createTopic<CmdVelPacket>(cmdVelInfo));
const velData = useSubscriber(cmdVelTopic.value);

const cmdX = computed(() => velData.value?.linear.x);
const cmdY = computed(() => velData.value?.linear.y);

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
  <div class="q-pa-none q-ma-none">
    <q-layout container style="height: 500px" class="shadow-2 rounded-borders">
      <div class="mock">
        cmd_vel x: {{ cmdX }}, y: {{ cmdY }}
        <p v-if="isConnected">Connected</p>
        <p v-else>Disconnected</p>
        <button @click="send">send!</button>
        <button
          @click="
            () => {
              connectRos('ws://localhost:9090');
            }
          "
        >
          connect!
        </button>
      </div>
    </q-layout>
    <Test />
  </div>
</template>

<style>
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
