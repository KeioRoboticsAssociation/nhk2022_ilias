<script lang="ts" setup>
import CmdVel from "../components/CmdVel.vue";
import Joy from "../components/Joy.vue";
</script>

<template>
  <q-page class="items-stretch row q-pa-xs">
    <div class="col-12 row wrap shelf">
      <CmdVel />
      <Joy />
      <CmdVel />
      <Joy />
      <CmdVel />
      <Joy />
      <CmdVel />
      <Joy />
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
