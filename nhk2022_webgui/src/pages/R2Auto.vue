<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12 row no-wrap justify-start items-start content-start">
      <div class="col-4">
        <p v-for="n in 1" :key="n">
          Lorem ipsum dolor sit amet consectetur adipisicing elit. Fugit nihil
          praesentium molestias a adipisci, dolore vitae odit, quidem
          consequatur optio voluptates asperiores pariatur eos numquam rerum
          delectus commodi perferendis voluptate?
        </p>
      </div>
      <div class="col column" style="height: 100%">
        <div class="col-7">
          <CmdVel />
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

<style lang="scss"></style>
