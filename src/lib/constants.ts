export const emojiMap: Record<string, string> = {
  lettuce: "\u{1F96C}",
  spinach: "\u{1F96C}",
  kale: "\u{1F96C}",
  arugula: "\u{1F331}",
  romaine: "\u{1F96C}",
  "mixed greens": "\u{1F96C}",
  quinoa: "\u{1F33E}",
  rice: "\u{1F35A}",
  chicken: "\u{1F357}",
  steak: "\u{1F969}",
  tofu: "\u{1F9C8}",
  shrimp: "\u{1F990}",
  salmon: "\u{1F41F}",
  egg: "\u{1F95A}",
  bacon: "\u{1F953}",
  tomato: "\u{1F345}",
  cucumber: "\u{1F952}",
  avocado: "\u{1F951}",
  corn: "\u{1F33D}",
  pepper: "\u{1FAD1}",
  "bell pepper": "\u{1FAD1}",
  onion: "\u{1F9C5}",
  carrot: "\u{1F955}",
  broccoli: "\u{1F966}",
  mushroom: "\u{1F344}",
  olive: "\u{1FAD2}",
  cheese: "\u{1F9C0}",
  croutons: "\u{1F35E}",
  nuts: "\u{1F95C}",
  "tortilla chips": "\u{1FAD3}",
  ranch: "\u{1F95B}",
  vinaigrette: "\u{1FAD7}",
  caesar: "\u{1F95B}",
  "balsamic vinaigrette": "\u{1FAD7}",
  "lemon tahini": "\u{1F34B}",
  "sesame ginger": "\u{1FAD0}",
};

const fallbackEmoji = "\u{1F957}";

export function getEmoji(name: string): string {
  const lower = name.toLowerCase();
  if (emojiMap[lower]) return emojiMap[lower];
  for (const [key, emoji] of Object.entries(emojiMap)) {
    if (lower.includes(key) || key.includes(lower)) return emoji;
  }
  return fallbackEmoji;
}

export const portionLimits: Record<string, number> = {
  base: 2,
  protein: 1,
  topping: 3,
  dressing: 1,
};

export const categoryLabels: Record<string, string> = {
  base: "Bases",
  protein: "Proteins",
  topping: "Toppings",
  dressing: "Dressings",
};

export const categoryOrder = ["base", "protein", "topping", "dressing"];
