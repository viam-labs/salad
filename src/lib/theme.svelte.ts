import {
  type ThemeId,
  applyThemeToDOM,
  copy,
  getCategoryLabelForTheme,
  getEmojiForTheme,
  parseThemeId,
} from "./theme";

export let currentTheme = $state<ThemeId>("salad");

export function setTheme(themeId: ThemeId): void {
  currentTheme = themeId;
  applyThemeToDOM(themeId);
}

export function getCopy() {
  return copy[currentTheme];
}

export function getCategoryLabel(category: string): string {
  return getCategoryLabelForTheme(category, currentTheme);
}

export function getEmoji(name: string): string {
  return getEmojiForTheme(name, currentTheme);
}

export function setThemeFromMachine(value: string): void {
  setTheme(parseThemeId(value));
}
