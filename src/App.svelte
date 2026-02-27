<script lang="ts">
  import { onMount } from "svelte";
  import { initConnection, fetchIngredients } from "./lib/robot";
  import type { Ingredient, AppScreen } from "./lib/types";
  import OrderingScreen from "./components/OrderingScreen.svelte";
  import BuildingScreen from "./components/BuildingScreen.svelte";
  import CompleteScreen from "./components/CompleteScreen.svelte";

  let screen: AppScreen = $state("loading");
  let ingredients: Ingredient[] = $state([]);
  let order: Record<string, number> = $state({});
  let error = $state("");

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
