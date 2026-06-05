import "./styles.css";
import { mount } from "svelte";
import App from "./App.svelte";
import { applyTheme } from "./lib/theme";

applyTheme();

mount(App, {
  target: document.getElementById("app")!,
});
