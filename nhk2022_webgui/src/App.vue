<template>
  <div class="allContainer">
    <iframe src="http://localhost:8001/rvizweb/" class="rvizweb"></iframe>
    <div class="mock">
      cmd_vel x: {{cmdX}}, y: {{cmdY}}
      <p v-if="isConnected">Connected</p>
      <p v-else>Disconnected</p>
      <button @click="send">send!</button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import * as roslib from 'roslib'
import { onMounted, ref } from 'vue'

const isConnected = ref<boolean>(false)
const cmdX = ref<number>(0)
const cmdY = ref<number>(0)

const ros = new roslib.Ros({
  url: 'ws://localhost:9090'
})

type cmdVelMsg = {
  linear: {
    x: number,
    y: number,
    z: number,
  },
  angular:{
    x: number,
    y: number,
    z: number
  }
}

const listener = new roslib.Topic<cmdVelMsg>({
  ros,
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist'
})

listener.subscribe(msg => {
  cmdX.value = msg.linear.x
  cmdY.value = msg.linear.y
})

function send () {
  console.log('send')
  const cmdVel = new roslib.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  })
  cmdVel.publish({
    linear: {
      x: 0.1,
      y: 0.2,
      z: 0.3
    },
    angular: {
      x: -0.1,
      y: -0.2,
      z: -0.3
    }
  })
}

onMounted(() => {
  ros.on('connection', () => {
    console.log('Connected to websocket server.')
    isConnected.value = true
  })

  ros.on('error', error => {
    console.log('Error connecting to websocket server: ', error)
  })

  ros.on('close', () => {
    console.log('Connection to websocket server closed.')
  })
})
</script>

<style lang="scss">
.rvizweb {
  display: block;
  height: 100vh;
  margin: 0;
  padding: 0;
  border: 0;

  flex-grow: 2;
}
.mock {
  flex-grow: 1;
}
.allContainer {
  display: flex;
  width: 100vw;
  height: 100vh;
}
body {
  margin: 0;
  padding: 0;
  width: 100vw;
  height: 100vh;
}
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin: 0;
}
</style>
