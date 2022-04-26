import { createRouter, createWebHistory } from "vue-router";
import Home from "./pages/Home.vue";
import R2Auto from "./pages/R2Auto.vue";
import R2Pile from "./pages/R2Pile.vue";

const routes = [
  { path: "/", name: "home", component: Home },
  { path: "/R2Auto", name: "R2Auto", component: R2Auto },
  { path: "/R2Pile", name: "R2Pile", component: R2Pile },
];

const router = createRouter({
  history: createWebHistory(), // HTML5 History モード
  routes,
});

export default router;
