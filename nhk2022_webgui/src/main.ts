import { createApp } from "vue";
import { Quasar } from "quasar";

import "quasar/src/css/index.sass";
import "@quasar/extras/material-icons/material-icons.css";

import App from "./App.vue";
import quasarUserOptions from "./quasar-user-options";

createApp(App).use(Quasar, quasarUserOptions).mount("#app");
