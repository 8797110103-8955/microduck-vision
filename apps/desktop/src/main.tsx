import { createApp } from "vue";
import { createPinia } from "pinia";
import { createTauriBackend, MemoryBackend } from "./bridge";
import { App } from "./features/app";
import { configureDesktopBackend } from "./store";
import "./styles/common.scss";

const hasTauriRuntime = "__TAURI_INTERNALS__" in globalThis || "__TAURI__" in globalThis;
configureDesktopBackend(hasTauriRuntime ? createTauriBackend() : new MemoryBackend([]));
createApp(App).use(createPinia()).mount(document.body);
