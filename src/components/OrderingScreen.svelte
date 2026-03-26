<script lang="ts">
  import { categoryOrder, portionLimits } from "../lib/constants";
  import type { Ingredient } from "../lib/types";
  import CategorySection from "./CategorySection.svelte";

  interface Props {
    ingredients: Ingredient[];
    onBuild: (order: Record<string, number>, name: string) => void;
  }

  let { ingredients, onBuild }: Props = $props();

  let order: Record<string, number> = $state({});
  let customerName = $state("");

  let totalItems = $derived(
    Object.values(order).reduce((sum, n) => sum + n, 0),
  );

  let cartNames = $derived(
    Object.entries(order)
      .filter(([, n]) => n > 0)
      .map(([name, n]) => (n > 1 ? `${name} x${n}` : name))
      .join(", "),
  );

  let grouped = $derived.by(() => {
    const g: Record<string, Ingredient[]> = {};
    for (const ing of ingredients) {
      if (!g[ing.category]) g[ing.category] = [];
      g[ing.category].push(ing);
    }
    return g;
  });

  function handleUpdate(name: string, count: number) {
    if (count <= 0) {
      const { [name]: _, ...rest } = order;
      order = rest;
    } else {
      order = { ...order, [name]: count };
    }
  }

  function handleBuild() {
    if (totalItems > 0) {
      onBuild({ ...order }, customerName.trim());
    }
  }
</script>

<div class="ordering-screen">
  <h1>Build Your Salad</h1>
  <div class="name-card">
    <label class="name-label" for="customer-name">What's your name?</label>
    <input
      id="customer-name"
      class="name-input"
      type="text"
      placeholder="e.g. Viam"
      maxlength="40"
      bind:value={customerName}
    />
  </div>

  {#each categoryOrder as cat (cat)}
    {@const items = grouped[cat]}
    {#if items?.length}
      <CategorySection
        category={cat}
        {items}
        {order}
        onUpdate={handleUpdate}
      />
    {/if}
  {/each}
</div>

<div class="cart-footer">
  <div class="cart-summary">
    {#if totalItems === 0}
      Your salad is <strong>empty</strong>
    {:else}
      <strong>{totalItems} item{totalItems > 1 ? "s" : ""}:</strong>
      {cartNames}
    {/if}
  </div>
  <button class="btn-build" disabled={totalItems === 0} onclick={handleBuild}>
    Build My Salad
  </button>
</div>
