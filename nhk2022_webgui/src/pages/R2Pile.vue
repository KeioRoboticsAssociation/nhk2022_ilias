<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
import Joy from "../components/Joy.vue";
import Arm from "../components/Arm.vue";

const { host } = window.location;
</script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12">
      <div class="row" style="height: 100%">
        <Joy class="item" />
        <Arm class="item" />
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
