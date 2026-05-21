import { defineConfig } from "vite";
import { svelte } from "@sveltejs/vite-plugin-svelte";
import glsl from "vite-plugin-glsl";

export default defineConfig({
  base: "./",
  plugins: [svelte(), glsl()],
  assetsInclude: ["**/*.hdr"],
  optimizeDeps: {
    esbuildOptions: {
      loader: {
        ".hdr": "dataurl",
        ".glsl": "text",
      },
    },
  },
});
