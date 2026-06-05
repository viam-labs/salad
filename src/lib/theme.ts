export type ThemeId = "salad" | "icecream";

const ICE_CREAM_ALIASES = new Set(["icecream", "ice-cream", "ice_cream"]);

export function parseThemeId(value: string): ThemeId {
  const normalized = value.trim().toLowerCase();
  if (ICE_CREAM_ALIASES.has(normalized)) return "icecream";
  return "salad";
}

export function applyThemeToDOM(themeId: ThemeId): void {
  document.documentElement.dataset.theme = themeId;
}

export const saladEmojiMap: Record<string, string> = {
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

export const iceCreamEmojiMap: Record<string, string> = {
  // flavors
  vanilla: "\u{1F366}",
  chocolate: "\u{1F36B}",
  strawberry: "\u{1F353}",
  "mint chip": "\u{1F33F}",
  mint: "\u{1F33F}",
  "cookie dough": "\u{1F36A}",
  "cookies and cream": "\u{1F36A}",
  caramel: "\u{1F36D}",
  "butter pecan": "\u{1F95C}",
  pistachio: "\u{1F95C}",
  "rocky road": "\u{1F36B}",
  coffee: "\u{2615}",
  mocha: "\u{2615}",
  neapolitan: "\u{1F368}",
  "birthday cake": "\u{1F382}",
  banana: "\u{1F34C}",
  mango: "\u{1F96D}",
  raspberry: "\u{1F352}",
  blueberry: "\u{1FAD0}",
  cherry: "\u{1F352}",
  lemon: "\u{1F34B}",
  coconut: "\u{1F965}",
  maple: "\u{1F9C8}",
  honey: "\u{1F36F}",
  "peanut butter": "\u{1F95C}",
  "black raspberry": "\u{1F352}",
  "salted caramel": "\u{1F36D}",
  matcha: "\u{1F375}",
  "s'mores": "\u{1F525}",
  // mix-ins
  brownie: "\u{1F36B}",
  "brownie bits": "\u{1F36B}",
  oreo: "\u{1F36A}",
  oreos: "\u{1F36A}",
  "chocolate chips": "\u{1F36B}",
  "peanut butter cups": "\u{1F36B}",
  "cookie pieces": "\u{1F36A}",
  "waffle cone pieces": "\u{1F9CA}",
  pretzel: "\u{1F968}",
  "pretzel pieces": "\u{1F968}",
  marshmallow: "\u{1F36F}",
  marshmallows: "\u{1F36F}",
  candy: "\u{1F36C}",
  "gummy bears": "\u{1F36C}",
  "m&ms": "\u{1F36C}",
  "reese's pieces": "\u{1F36B}",
  // toppings
  sprinkles: "\u{2728}",
  "rainbow sprinkles": "\u{2728}",
  "chocolate sprinkles": "\u{2728}",
  "whipped cream": "\u{1F9CB}",
  "maraschino cherry": "\u{1F352}",
  nuts: "\u{1F95C}",
  peanuts: "\u{1F95C}",
  walnuts: "\u{1F95C}",
  almonds: "\u{1F95C}",
  pecans: "\u{1F95C}",
  "crushed nuts": "\u{1F95C}",
  "banana slices": "\u{1F34C}",
  "fresh strawberries": "\u{1F353}",
  "cookie crumbles": "\u{1F36A}",
  "coconut flakes": "\u{1F965}",
  granola: "\u{1F96F}",
  "waffle cone": "\u{1F9CA}",
  "sugar cone": "\u{1F9CA}",
  cone: "\u{1F9CA}",
  // sauces
  "hot fudge": "\u{1F36B}",
  "chocolate sauce": "\u{1F36B}",
  "chocolate syrup": "\u{1F36B}",
  "caramel sauce": "\u{1F36D}",
  "strawberry sauce": "\u{1F353}",
  "strawberry syrup": "\u{1F353}",
  "raspberry sauce": "\u{1F352}",
  "blueberry sauce": "\u{1FAD0}",
  "peanut butter sauce": "\u{1F95C}",
  "marshmallow sauce": "\u{1F36F}",
  butterscotch: "\u{1F36D}",
  "maple syrup": "\u{1F9C8}",
  "honey drizzle": "\u{1F36F}",
  "caramel drizzle": "\u{1F36D}",
};

const fallbackEmoji: Record<ThemeId, string> = {
  salad: "\u{1F957}",
  icecream: "\u{1F366}",
};

export function getEmojiForTheme(name: string, themeId: ThemeId): string {
  const map = themeId === "icecream" ? iceCreamEmojiMap : saladEmojiMap;
  const lower = name.toLowerCase();
  if (map[lower]) return map[lower];
  for (const [key, emoji] of Object.entries(map)) {
    if (lower.includes(key) || key.includes(lower)) return emoji;
  }
  return fallbackEmoji[themeId];
}

export const categoryLabelsByTheme: Record<ThemeId, Record<string, string>> = {
  salad: {
    base: "Bases",
    protein: "Proteins",
    topping: "Toppings",
    dressing: "Dressings",
  },
  icecream: {
    base: "Flavors",
    protein: "Mix-ins",
    topping: "Toppings",
    dressing: "Sauces",
  },
};

export function getCategoryLabelForTheme(
  category: string,
  themeId: ThemeId,
): string {
  return categoryLabelsByTheme[themeId][category] ?? category;
}

export const copy = {
  salad: {
    orderTitle: "Build Your Salad",
    emptyCartLabel: "Your salad is",
    buildButton: "Build My Salad",
    buildingTitle: (name: string) =>
      name ? `Building ${name}'s Salad\u2026` : "Building Your Salad\u2026",
    completeEmoji: "\u{1F957}",
    completeTitle: (name: string) =>
      name ? `${name}'s Salad is Ready!` : "Your Salad is Ready!",
  },
  icecream: {
    orderTitle: "Build Your Sundae",
    emptyCartLabel: "Your sundae is",
    buildButton: "Build My Sundae",
    buildingTitle: (name: string) =>
      name ? `Building ${name}'s Sundae\u2026` : "Building Your Sundae\u2026",
    completeEmoji: "\u{1F366}",
    completeTitle: (name: string) =>
      name ? `${name}'s Sundae is Ready!` : "Your Sundae is Ready!",
  },
} as const;
