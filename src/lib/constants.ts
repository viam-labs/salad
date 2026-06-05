import { getEmojiForTheme } from "./theme";

export function getEmoji(name: string): string {
  return getEmojiForTheme(name);
}

export const portionLimits: Record<string, number> = {
  base: 2,
  protein: 1,
  topping: 3,
  dressing: 1,
};

export const categoryOrder = ["base", "protein", "topping", "dressing"];
