export type ThemeId = "salad" | "icecream" | "mediterranean";

const ICE_CREAM_ALIASES = new Set(["icecream", "ice-cream", "ice_cream"]);
const MEDITERRANEAN_ALIASES = new Set([
  "mediterranean",
  "med",
  "mezze",
]);

export function parseThemeId(value: string): ThemeId {
  const normalized = value.trim().toLowerCase();
  if (ICE_CREAM_ALIASES.has(normalized)) return "icecream";
  if (MEDITERRANEAN_ALIASES.has(normalized)) return "mediterranean";
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

export const mediterraneanEmojiMap: Record<string, string> = {
  // bases
  pita: "\u{1F9C6}",
  "pita bread": "\u{1F9C6}",
  lavash: "\u{1F9C6}",
  flatbread: "\u{1F9C6}",
  couscous: "\u{1F35A}",
  bulgur: "\u{1F33E}",
  rice: "\u{1F35A}",
  quinoa: "\u{1F33E}",
  greens: "\u{1F96C}",
  arugula: "\u{1F331}",
  spinach: "\u{1F96C}",
  romaine: "\u{1F96C}",
  "mixed greens": "\u{1F96C}",
  // proteins
  falafel: "\u{1F9C6}",
  chicken: "\u{1F357}",
  lamb: "\u{1F969}",
  shrimp: "\u{1F990}",
  salmon: "\u{1F41F}",
  chickpeas: "\u{1FAD8}",
  halloumi: "\u{1F9C0}",
  egg: "\u{1F95A}",
  // toppings
  tomato: "\u{1F345}",
  cucumber: "\u{1F952}",
  olive: "\u{1FAD2}",
  olives: "\u{1FAD2}",
  "kalamata olives": "\u{1FAD2}",
  feta: "\u{1F9C0}",
  "feta cheese": "\u{1F9C0}",
  "red onion": "\u{1F9C5}",
  onion: "\u{1F9C5}",
  "roasted pepper": "\u{1FAD1}",
  pepper: "\u{1FAD1}",
  artichoke: "\u{1F331}",
  hummus: "\u{1F963}",
  tzatziki: "\u{1FAD5}",
  "sun-dried tomato": "\u{1F345}",
  capers: "\u{1FAD2}",
  mint: "\u{1F33F}",
  parsley: "\u{1F33F}",
  // dressings
  "lemon vinaigrette": "\u{1F34B}",
  tahini: "\u{1F34B}",
  "tahini dressing": "\u{1F34B}",
  harissa: "\u{1F336}",
  "tzatziki sauce": "\u{1FAD5}",
  "olive oil": "\u{1FAD2}",
  "lemon herb": "\u{1F34B}",
  "garlic lemon": "\u{1F9C4}",
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
  mediterranean: "\u{1F9C6}",
};

export function getEmojiForTheme(name: string, themeId: ThemeId): string {
  const map =
    themeId === "icecream"
      ? iceCreamEmojiMap
      : themeId === "mediterranean"
        ? mediterraneanEmojiMap
        : saladEmojiMap;
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
  mediterranean: {
    base: "Bases",
    protein: "Proteins",
    topping: "Toppings",
    dressing: "Dressings",
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
  mediterranean: {
    orderTitle: "Build Your Mezze Plate",
    emptyCartLabel: "Your mezze plate is",
    buildButton: "Build My Mezze Plate",
    buildingTitle: (name: string) =>
      name
        ? `Building ${name}'s Mezze Plate\u2026`
        : "Building Your Mezze Plate\u2026",
    completeEmoji: "\u{1F9C6}",
    completeTitle: (name: string) =>
      name ? `${name}'s Mezze Plate is Ready!` : "Your Mezze Plate is Ready!",
  },
} as const;
