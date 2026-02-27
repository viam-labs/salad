<script lang="ts">
  import { onMount } from "svelte";
  import { initConnection, fetchIngredients, getCameraStream, removeCameraStream } from "./lib/robot";
  import type { Ingredient, AppScreen } from "./lib/types";
  import OrderingScreen from "./components/OrderingScreen.svelte";
  import BuildingScreen from "./components/BuildingScreen.svelte";
  import CompleteScreen from "./components/CompleteScreen.svelte";

  let screen: AppScreen = $state("loading");
  let ingredients: Ingredient[] = $state([]);
  let order: Record<string, number> = $state({});
  let error = $state("");
  let showCamera = $state(false);
  let cameraVideo: HTMLVideoElement = $state()!;

  async function openCamera() {
    showCamera = true;
    try {
      const stream = await getCameraStream("overhead-webcam");
      cameraVideo.srcObject = stream;
    } catch (err) {
      console.error("Camera stream error:", err);
    }
  }

  async function closeCamera() {
    cameraVideo.srcObject = null;
    showCamera = false;
    await removeCameraStream("overhead-webcam");
  }

  onMount(async () => {
    try {
      await initConnection();
      ingredients = await fetchIngredients();
      screen = "ordering";
    } catch (err) {
      error = err instanceof Error ? err.message : String(err);
      screen = "error";
    }
  });

  function handleBuild(newOrder: Record<string, number>) {
    order = newOrder;
    screen = "building";
  }

  function handleComplete() {
    screen = "complete";
  }

  function handleNewOrder() {
    order = {};
    screen = "ordering";
  }
</script>

{#if screen !== "loading" && screen !== "error" && screen !== "building"}
  <button class="camera-fab" onclick={openCamera}>📷</button>
{/if}

{#if showCamera}
  <div
    class="camera-modal-backdrop"
    role="presentation"
    onclick={closeCamera}
    onkeydown={(e) => e.key === "Escape" && closeCamera()}
  >
    <div
      class="camera-modal"
      role="dialog"
      aria-modal="true"
      aria-label="Camera stream"
      tabindex="-1"
      onclick={(e) => e.stopPropagation()}
      onkeydown={(e) => e.stopPropagation()}
    >
      <button class="camera-modal-close" onclick={closeCamera}>✕</button>
      <video class="camera-modal-video" bind:this={cameraVideo} autoplay playsinline></video>
    </div>
  </div>
{/if}

{#if screen === "loading"}
  <div class="loading">Connecting&hellip;</div>
{:else if screen === "error"}
  <div class="error-screen">
    <h1>Could not connect</h1>
    <p>{error}</p>
  </div>
{:else if screen === "ordering"}
  <OrderingScreen {ingredients} onBuild={handleBuild} />
{:else if screen === "building"}
  <BuildingScreen {order} onComplete={handleComplete} onStopped={handleNewOrder} />
{:else if screen === "complete"}
  <CompleteScreen onNewOrder={handleNewOrder} />
{/if}
