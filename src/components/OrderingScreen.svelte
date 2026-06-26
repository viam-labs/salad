<script lang="ts">
  import { categoryOrder } from "../lib/constants";
  import { getCopy } from "../lib/theme.svelte";
  import type { Ingredient } from "../lib/types";
  import CategorySection from "./CategorySection.svelte";

  interface Props {
    ingredients: Ingredient[];
    onBuild: (order: Record<string, number>, name: string) => void;
  }

  let { ingredients, onBuild }: Props = $props();

  let text = $derived(getCopy());

  let order: Record<string, number> = $state({});
  let customerName = $state("");
  let nameError = $state(false);

  let trimmedName = $derived(customerName.trim());
  let nameValid = $derived(trimmedName.length > 0);

  let totalItems = $derived(
    Object.values(order).reduce((sum, n) => sum + n, 0),
  );

  let canBuild = $derived(totalItems > 0 && nameValid);

  // True once the cart has items but no name yet: the build button is
  // disabled in this state, so surface why instead of leaving it inert.
  let needsName = $derived(totalItems > 0 && !nameValid);

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
    if (!nameValid) {
      nameError = true;
      return;
    }
    if (canBuild) {
      onBuild({ ...order }, trimmedName);
    }
  }
</script>

<div class="ordering-screen">
  <h1>{text.orderTitle}</h1>
  <div class="name-card" class:needs-name={needsName}>
    <label class="name-label" for="customer-name">
      What's your name? <span class="name-required" aria-hidden="true">*</span>
    </label>
    <input
      id="customer-name"
      class="name-input"
      class:invalid={nameError && !nameValid}
      class:needs-name={needsName}
      type="text"
      placeholder="e.g. Viam"
      maxlength="40"
      required
      aria-required="true"
      aria-invalid={nameError && !nameValid}
      bind:value={customerName}
      oninput={() => { if (nameValid) nameError = false; }}
    />
    {#if nameError && !nameValid}
      <div class="name-error" role="alert">Please enter your name to build.</div>
    {/if}
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
      {text.emptyCartLabel} <strong>empty</strong>
    {:else}
      <strong>{totalItems} item{totalItems > 1 ? "s" : ""}:</strong>
      {cartNames}
    {/if}
  </div>
  <div class="cart-actions">
    {#if needsName}
      <span class="build-hint" role="status">
        Add your name above to build <span aria-hidden="true">↑</span>
      </span>
    {/if}
    <button class="btn-build" disabled={!canBuild} onclick={handleBuild}>
      {text.buildButton}
    </button>
  </div>
</div>
