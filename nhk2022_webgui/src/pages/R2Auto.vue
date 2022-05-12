<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
import AutoPath from "../components/AutoPath.vue";
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12 row no-wrap justify-start items-start content-start">
      <div class="col-4" style="height: 100%">
        <iframe
          src="http://localhost:8001/rvizweb"
          style="display: block; width: 100%; height: 100%"
        ></iframe>
      </div>
      <div class="col column" style="height: 100%">
        <div class="col-7 row shelf">
          <CmdVel />
          <AutoPath />
        </div>
        <div class="col">
          <p v-for="n in 3" :key="n">
            Lorem ipsum dolor sit amet consectetur adipisicing elit. Fugit nihil
            praesentium molestias a adipisci, dolore vitae odit, quidem
            consequatur optio voluptates asperiores pariatur eos numquam rerum
            delectus commodi perferendis voluptate?
          </p>
        </div>
      </div>
    </div>
  </q-page>
</template>

<style lang="scss" scoped>
.shelf {
  * {
    width: calc(98% / 3);
    height: 98%;
    margin: 0.1rem;
  }
}
</style>
