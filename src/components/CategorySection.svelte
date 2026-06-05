<script lang="ts">
  import { portionLimits } from "../lib/constants";
  import { getCategoryLabel } from "../lib/theme";
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
  let label = $derived(getCategoryLabel(category));
  let categoryTotal = $derived(
    items.reduce((sum, ing) => sum + (order[ing.name] ?? 0), 0),
  );
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
        {categoryTotal}
        onUpdate={(count) => onUpdate(ing.name, count)}
      />
    {/each}
  </div>
</div>
