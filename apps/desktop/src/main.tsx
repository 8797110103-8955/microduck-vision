import { createApp } from "vue";
import { createPinia } from "pinia";
import { App } from "./features/app";
import "./styles/common.scss";

createApp(App).use(createPinia()).mount(document.body);
