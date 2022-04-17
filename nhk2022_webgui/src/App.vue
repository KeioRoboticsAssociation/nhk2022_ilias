<script lang="ts" setup>
import { onMounted, ref } from "vue";

// https://stackoverflow.com/questions/69491226/after-using-vitevue3-to-build-the-project-and-importing-roslibjs-an-error-is-r
import "roslib/build/roslib";

const roslib = window.ROSLIB;

const isConnected = ref<boolean>(false);
const cmdX = ref<number>(0);
const cmdY = ref<number>(0);

const ros = new roslib.Ros({
  url: "ws://localhost:9090",
});

type CmdVelMsg = {
  linear: {
    x: number;
    y: number;
    z: number;
  };
  angular: {
    x: number;
    y: number;
    z: number;
  };
};

const listener = new roslib.Topic<CmdVelMsg>({
  ros,
  name: "/cmd_vel",
  messageType: "geometry_msgs/Twist",
});

listener.subscribe((msg) => {
  cmdX.value = msg.linear.x;
  cmdY.value = msg.linear.y;
});

function send() {
  console.log("send");
  const cmdVel = new roslib.Topic({
    ros,
    name: "/cmd_vel",
    messageType: "geometry_msgs/Twist",
  });
  cmdVel.publish({
    linear: {
      x: 0.1,
      y: 0.2,
      z: 0.3,
    },
    angular: {
      x: -0.1,
      y: -0.2,
      z: -0.3,
    },
  });
}

onMounted(() => {
  ros.on("connection", () => {
    console.log("Connected to websocket server.");
    isConnected.value = true;
  });

  ros.on("error", (error) => {
    console.log("Error connecting to websocket server: ", error);
  });

  ros.on("close", () => {
    console.log("Connection to websocket server closed.");
  });
});
</script>

<template>
  <div class="q-pa-none q-ma-none">
    <q-layout container style="height: 500px" class="shadow-2 rounded-borders">
      <q-header reveal class="bg-black">
        <q-toolbar>
          <q-toolbar-title>Header</q-toolbar-title>
        </q-toolbar>
      </q-header>
      <div class="mock">
        cmd_vel x: {{ cmdX }}, y: {{ cmdY }}
        <p v-if="isConnected">Connected</p>
        <p v-else>Disconnected</p>
        <button @click="send">send!</button>
      </div>
    </q-layout>
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
