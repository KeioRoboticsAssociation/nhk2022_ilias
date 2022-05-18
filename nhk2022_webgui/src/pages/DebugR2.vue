<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
import Joy from "../components/Joy.vue";
import Odom from "../components/Odom.vue";
import AutoPath from "../components/AutoPath.vue";
import Arm from "../components/Arm.vue";
</script>

<template>
  <q-page class="items-stretch row q-pa-xs">
    <div class="col-12 row wrap shelf">
      <CmdVel />
      <Joy />
      <Odom />
      <AutoPath />
      <Arm />
    </div>
  </q-page>
</template>

<style lang="scss" scoped>
.shelf {
  * {
    width: calc(98% / 5);
    height: 48%;
    margin: 0.1rem;
  }
}
</style>
