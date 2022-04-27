<script lang="ts" setup></script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12 row no-wrap justify-start items-start content-start">
      <div class="col column links">
        <q-btn>
          <router-link to="/R2Pile">R2Pile</router-link>
        </q-btn>
        <q-btn>
          <router-link to="/R2Auto">R2Auto</router-link>
        </q-btn>
        <q-btn>
          <router-link to="/DebugR2">DebugR2</router-link>
        </q-btn>
      </div>
      <div class="col"></div>
    </div>
  </q-page>
</template>

<style lang="scss" scoped>
.right-field {
  height: 100vh;
}
.links {
  * {
    display: black;
    text-decoration: none;
    width: 100%;
    height: 100%;
  }
}
</style>
