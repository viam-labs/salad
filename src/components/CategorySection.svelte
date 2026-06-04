<script lang="ts">
  import { getCategoryLabel } from "../lib/theme.svelte";
  import type { Ingredient } from "../lib/types";
  import IngredientTile from "./IngredientTile.svelte";

  interface Props {
    category: string;
    items: Ingredient[];
    order: Record<string, number>;
    onUpdate: (name: string, count: number) => void;
  }

  let { category, items, order, onUpdate }: Props = $props();

  let label = $derived(getCategoryLabel(category));
</script>

<div class="category-section">
  <h2>{label}</h2>
  <div class="ingredient-grid">
    {#each items as ing (ing.name)}
      <IngredientTile
        name={ing.name}
        count={order[ing.name] ?? 0}
        onUpdate={(count) => onUpdate(ing.name, count)}
      />
    {/each}
  </div>
</div>
