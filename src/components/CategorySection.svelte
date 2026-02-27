<script lang="ts">
  import { categoryLabels, portionLimits } from "../lib/constants";
  import type { Ingredient } from "../lib/types";
  import IngredientTile from "./IngredientTile.svelte";

  interface Props {
    category: string;
    items: Ingredient[];
    order: Record<string, number>;
    onUpdate: (name: string, count: number) => void;
  }

  let { category, items, order, onUpdate }: Props = $props();

  let limit = $derived(portionLimits[category] ?? 1);
  let label = $derived(categoryLabels[category] ?? category);
</script>

<div class="category-section">
  <h2>{label}</h2>
  <div class="category-limit">Up to {limit} portion{limit > 1 ? "s" : ""}</div>
  <div class="ingredient-grid">
    {#each items as ing (ing.name)}
      <IngredientTile
        name={ing.name}
        count={order[ing.name] ?? 0}
        {limit}
        onUpdate={(count) => onUpdate(ing.name, count)}
      />
    {/each}
  </div>
</div>
