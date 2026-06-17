import {
  type ThemeId,
  applyThemeToDOM,
  copy,
  getCategoryLabelForTheme,
  getEmojiForTheme,
  parseThemeId,
} from "./theme";

const themeState = $state<{ current: ThemeId }>({ current: "salad" });

export function getCurrentTheme(): ThemeId {
  return themeState.current;
}

export function setTheme(themeId: ThemeId): void {
  themeState.current = themeId;
  applyThemeToDOM(themeId);
}

export function getCopy() {
  return copy[themeState.current];
}

export function getCategoryLabel(category: string): string {
  return getCategoryLabelForTheme(category, themeState.current);
}

export function getEmoji(name: string): string {
  return getEmojiForTheme(name, themeState.current);
}

export function setThemeFromMachine(value: string): void {
  setTheme(parseThemeId(value));
}
