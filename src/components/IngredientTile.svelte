<script lang="ts">
  import { getEmoji } from "../lib/constants";

  interface Props {
    name: string;
    count: number;
    limit: number;
    onUpdate: (count: number) => void;
  }

  let { name, count, limit, onUpdate }: Props = $props();

  let emoji = $derived(getEmoji(name));
  let selected = $derived(count > 0);
</script>

<div class="ingredient-tile" class:selected>
  <div class="tile-emoji">{emoji}</div>
  <div class="tile-name">{name}</div>
  <div class="stepper">
    <button disabled={count <= 0} onclick={() => onUpdate(count - 1)}>&minus;</button>
    <span class="stepper-count">{count}</span>
    <button disabled={count >= limit} onclick={() => onUpdate(count + 1)}>+</button>
  </div>
</div>
