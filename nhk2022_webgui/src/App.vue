<template>
  <div class="allContainer">
    <iframe src="http://localhost:8001/rvizweb/" class="rvizweb"></iframe>
    <div class="mock"></div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'App'
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
