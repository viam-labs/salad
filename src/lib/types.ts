export interface Ingredient {
  name: string;
  grams_per_serving: number;
  category: string;
}

export type AppScreen = "loading" | "ordering" | "building" | "complete" | "error" | "setup";

export interface ZoneMesh {
  vertices: [number, number, number][];
  faces: [number, number, number][];
}

export interface Zone {
  id: number;
  min_x: number;
  max_x: number;
  min_y: number;
  max_y: number;
  mesh: ZoneMesh;
}

export interface ZonesResult {
  source_mesh: string;
  generated_at: string;
  zones: Zone[];
}

export interface SetupResult {
  pcd: Uint8Array;
  zones: ZonesResult;
}
