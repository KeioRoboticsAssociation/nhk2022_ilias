import { createApp } from "vue";
import { Quasar } from "quasar";
import router from "./router";

import "quasar/src/css/index.sass";
import "@quasar/extras/material-icons/material-icons.css";

import App from "./App.vue";
import quasarUserOptions from "./quasar-user-options";

createApp(App).use(Quasar, quasarUserOptions).use(router).mount("#app");

document.body.addEventListener(
  "touchstart",
  (e: TouchEvent) => {
    e.preventDefault();
  },
  { passive: false }
);
document.body.addEventListener(
  "touchmove",
  (e: TouchEvent) => {
    e.preventDefault();
  },
  { passive: false }
);
