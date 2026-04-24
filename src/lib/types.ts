export interface Ingredient {
  name: string;
  grams_per_serving: number;
  category: string;
}

export type AppScreen = "loading" | "ordering" | "building" | "complete" | "error";

export type BuildState =
  | "idle"
  | "preparing"
  | "adding"
  | "delivering"
  | "complete"
  | "stopped"
  | "failed";

export interface BuildStatus {
  state: BuildState;
  progress: number;
  customer_name?: string;
  current_ingredient?: string;
  failure_reason?: string;
  interrupted_at?: string;
  warnings?: string[];
  elapsed_seconds?: number;
  eta_seconds?: number;
}
