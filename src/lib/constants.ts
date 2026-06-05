export { getEmoji } from "./theme.svelte";

export const portionLimits: Record<string, number> = {
  base: 2,
  protein: 1,
  topping: 3,
  dressing: 1,
};

export const categoryOrder = ["base", "protein", "topping", "dressing"];
