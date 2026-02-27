export interface Ingredient {
  name: string;
  grams_per_serving: number;
  category: string;
}

export type AppScreen = "loading" | "ordering" | "building" | "complete" | "error";
