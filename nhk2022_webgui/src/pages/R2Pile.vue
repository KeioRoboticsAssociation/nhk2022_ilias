<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
import Joy from "../components/Joy.vue";
import Arm from "../components/Arm.vue";

const { host } = window.location;
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12 row no-wrap justify-start items-start content-start">
      <div class="col-6 q-pa-sm">
        <img
          :src="`http://${host}:8080/stream?topic=/head_camera/image_raw&type=ros_compressed`"
          alt=""
          class="cam"
        />
      </div>
      <div class="col column" style="height: 100%">
        <div class="col-6 row justify-around">
          <CmdVel class="item" />
          <Joy class="item" />
        </div>
        <div class="col-6 row justify-end">
          <Arm class="item" />
        </div>
      </div>
    </div>
  </q-page>
</template>

<style lang="scss" scoped>
.cam {
  display: block;
  width: 100%;
  height: 100%;
  object-fit: cover;
}
.item {
  width: calc(99% / 2);
  height: 99%;
}
</style>
