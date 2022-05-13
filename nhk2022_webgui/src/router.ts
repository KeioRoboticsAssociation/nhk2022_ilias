import { createRouter, createWebHashHistory } from "vue-router";
import Home from "./pages/Home.vue";
import R2Auto from "./pages/R2Auto.vue";
import R2Pile from "./pages/R2Pile.vue";
import DebugR2 from "./pages/DebugR2.vue";

const routes = [
  { path: "/", name: "home", component: Home },
  { path: "/R2Auto", name: "R2Auto", component: R2Auto },
  { path: "/R2Pile", name: "R2Pile", component: R2Pile },
  { path: "/DebugR2", name: "DebugR2", component: DebugR2 },
];

const router = createRouter({
  history: createWebHashHistory(), // HTML5 History モード
  routes,
});

export default router;
