<script lang="ts" setup></script>

<template>
  <q-page padding class="items-stretch row">
    <div class="col-12 row no-wrap justify-start items-start content-start">
      <div class="col column">
        <q-btn>
          <router-link to="/R2Pile" class="link">R2Pile</router-link>
        </q-btn>
        <q-btn>
          <router-link to="/R2Auto" class="link">R2Auto</router-link>
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
.link {
  display: black;
  text-decoration: none;
  width: 100%;
  height: 100%;
}
</style>
