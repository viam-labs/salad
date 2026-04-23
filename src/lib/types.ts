export interface Ingredient {
  name: string;
  grams_per_serving: number;
  category: string;
}

export interface QueueOrder {
  id: string;
  customer_name: string;
  status: "queued" | "building" | "complete" | "cancelled" | "failed";
  position?: number;
  progress?: number;
  current_step?: string;
  estimated_wait_sec?: number;
  estimated_remaining_sec?: number;
}

export interface QueueSnapshot {
  orders: QueueOrder[];
  avg_build_duration_sec: number;
}

export type AppScreen = "loading" | "ordering" | "queue" | "building" | "complete" | "error";
