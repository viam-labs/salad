<script lang="ts">
  import { onMount } from "svelte";
  import { initConnection, fetchIngredients, getCameraStream, queueOrder } from "./lib/robot";
  import type { Ingredient, AppScreen } from "./lib/types";
  import OrderingScreen from "./components/OrderingScreen.svelte";
  import QueueBoard from "./components/QueueBoard.svelte";
  import BuildingScreen from "./components/BuildingScreen.svelte";
  import CompleteScreen from "./components/CompleteScreen.svelte";

  let screen: AppScreen = $state("loading");
  let ingredients: Ingredient[] = $state([]);
  let order: Record<string, number> = $state({});
  let customerName = $state("");
  let orderId: string | null = $state(null);
  let error = $state("");
  let showCamera = $state(false);

  // Check for shared display mode.
  const displayOnly = new URLSearchParams(window.location.search).has("display");

  function attachStream(node: HTMLVideoElement) {
    node.srcObject = getCameraStream();
  }

  function openCamera() { showCamera = true; }
  function closeCamera() { showCamera = false; }

  onMount(async () => {
    try {
      await initConnection();
      if (displayOnly) {
        screen = "queue";
        return;
      }
      ingredients = await fetchIngredients();
      screen = "ordering";
    } catch (err) {
      error = err instanceof Error ? err.message : String(err);
      screen = "error";
    }
  });

  async function handleBuild(newOrder: Record<string, number>, name: string) {
    order = newOrder;
    customerName = name;
    try {
      const result = await queueOrder(newOrder, name);
      orderId = result.order_id;
      screen = "queue";
    } catch (err) {
      console.error("Failed to queue order:", err);
    }
  }

  function handleBuilding(name: string) {
    customerName = name;
    screen = "building";
  }

  function handleComplete() {
    screen = "complete";
  }

  function handleNewOrder() {
    order = {};
    orderId = null;
    screen = "ordering";
  }
</script>

{#if screen !== "loading" && screen !== "error"}
  <video use:attachStream autoplay playsinline style="display:none"></video>
{/if}

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
      <video class="camera-modal-video" use:attachStream autoplay playsinline></video>
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
{:else if screen === "queue"}
  <QueueBoard
    {orderId}
    {displayOnly}
    onBuilding={handleBuilding}
    onComplete={handleComplete}
  />
{:else if screen === "building"}
  <BuildingScreen {order} {customerName} onComplete={handleComplete} onStopped={handleNewOrder} />
{:else if screen === "complete"}
  <CompleteScreen {customerName} onNewOrder={handleNewOrder} />
{/if}
