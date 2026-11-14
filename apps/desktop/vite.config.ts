import { fileURLToPath, URL } from "node:url";
import { defineConfig } from "vitest/config";
import vueJsx from "@vitejs/plugin-vue-jsx";

export default defineConfig({
  plugins: [vueJsx()],
  resolve: { alias: { "@": fileURLToPath(new URL("./src", import.meta.url)) } },
  clearScreen: false,
  server: { port: 5173, strictPort: true, watch: { ignored: ["**/src-tauri/**"] } },
  build: { chunkSizeWarningLimit: Number.POSITIVE_INFINITY },
  test: { environment: "happy-dom" },
});
