// Dashboard data client: each function builds an MQL pipeline run server-side
// via the Data API. RESOURCE_NAME is the sensor's config "name", not its model.
import * as VIAM from "@viamrobotics/sdk";
import { BSON } from "bson";

export const RESOURCE_NAME = "salad-events";

// Event-type strings; mirror the Go constants in events/event.go.
export const EventType = {
  BuildStart: "build_start",
  BuildComplete: "build_complete",
  BuildFailed: "build_failed",
  BuildStopped: "build_stopped",
  IngredientStart: "ingredient_start",
  IngredientComplete: "ingredient_complete",
  GrabAttempt: "grab_attempt",
  DressingPour: "dressing_pour",
  SetupComplete: "setup_complete",
  SetupFailed: "setup_failed",
} as const;

export interface DataContext {
  client: VIAM.ViamClient;
  orgId: string;
  locationId: string;
}

// ── Result types ────────────────────────────────────────────────

export interface BuildSummary {
  buildId: string;
  customerName: string;        // display-cased
  customerNameKey: string;     // normalized join key
  startTime: Date;
  endTime: Date;
  durationMs: number;
  outcome: "ok" | "failed" | "stopped";
  errorMessage: string;
  totalServings: number;
}

export interface DailyBuildCount {
  day: Date;
  ok: number;
  failed: number;
  stopped: number;
}

export interface IngredientStat {
  name: string;
  attempts: number;
  successes: number;
  successRate: number;
  avgGramsError: number;
  emptyBinEvents: number;
}

export interface CustomerStat {
  name: string;     // display
  nameKey: string;  // normalized
  buildCount: number;
}

export interface TrendPoint {
  bucketStart: Date;
  value: number;
  sampleSize: number;
}

export interface DurationTrendPoint {
  bucketStart: Date;
  p50: number;
  p95: number;
  sampleSize: number;
}

export interface IngredientTrendPoint extends TrendPoint {
  ingredientName: string;
}

export interface GrabAttempt {
  buildId: string;
  ingredientName: string;
  zoneId: number;
  attemptIndex: number;
  depthOffsetMm: number;
  weightChangeG: number;
  outcome: string;
  motionPlanningFailure: boolean;
  errorMessage: string;
  durationMs: number;
  timestamp: Date;
}

export type BucketUnit = "day" | "week";

// ── Internals ──────────────────────────────────────────────────

async function runMQL<T>(
  ctx: DataContext,
  stages: Record<string, unknown>[],
): Promise<T[]> {
  const serialized = stages.map((s) => BSON.serialize(s));
  return (await ctx.client.dataClient.tabularDataByMQL(
    ctx.orgId,
    serialized as unknown as Parameters<
      typeof ctx.client.dataClient.tabularDataByMQL
    >[1],
  )) as T[];
}

// Browser timezone, resolved once per session.
const TZ = ((): string => {
  try {
    return Intl.DateTimeFormat().resolvedOptions().timeZone || "UTC";
  } catch {
    return "UTC";
  }
})();

// Build-end events, used wherever we count or summarize finished builds.
const BUILD_END_TYPES = [
  EventType.BuildComplete,
  EventType.BuildFailed,
  EventType.BuildStopped,
];

function sinceDate(days: number): Date {
  return new Date(Date.now() - days * 24 * 60 * 60 * 1000);
}

function toDate(v: unknown): Date {
  if (v instanceof Date) return v;
  if (typeof v === "string" || typeof v === "number") return new Date(v);
  return new Date(0);
}

// dateTrunc buckets time_received into the given unit in the browser timezone.
function dateTrunc(unit: BucketUnit) {
  return { $dateTrunc: { date: "$time_received", unit, timezone: TZ } };
}

function baseMatch(ctx: DataContext, extra: Record<string, unknown> = {}) {
  return {
    location_id: ctx.locationId,
    component_name: RESOURCE_NAME,
    ...extra,
  };
}

// eventMatch builds a $match for one or more event types, optionally limited to
// the last `sinceDays` days.
function eventMatch(
  ctx: DataContext,
  eventType: string | readonly string[],
  sinceDays?: number,
) {
  const extra: Record<string, unknown> = {
    "data.readings.event_type": Array.isArray(eventType)
      ? { $in: eventType }
      : eventType,
  };
  if (sinceDays !== undefined) {
    extra.time_received = { $gte: sinceDate(sinceDays) };
  }
  return baseMatch(ctx, extra);
}

// Map a build-end event_type string to our BuildSummary outcome enum.
function outcomeFromEventType(t: unknown): BuildSummary["outcome"] {
  switch (t) {
    case EventType.BuildComplete:
      return "ok";
    case EventType.BuildStopped:
      return "stopped";
    default:
      return "failed";
  }
}

// ── Queries: current-window aggregates ─────────────────────────

export function formatDay(day: Date): string {
  return day.toLocaleDateString(undefined, {
    weekday: "short",
    month: "short",
    day: "numeric",
  });
}

export async function loadDailyBuildCounts(
  ctx: DataContext,
  sinceDays: number,
): Promise<DailyBuildCount[]> {
  const rows = await runMQL<{
    day: Date | string;
    event_type: string;
    value: number;
  }>(ctx, [
    { $match: eventMatch(ctx, BUILD_END_TYPES, sinceDays) },
    {
      $group: {
        _id: {
          day: dateTrunc("day"),
          event_type: "$data.readings.event_type",
        },
        value: { $sum: 1 },
      },
    },
    {
      $project: {
        _id: 0,
        day: "$_id.day",
        event_type: "$_id.event_type",
        value: 1,
      },
    },
  ]);

  const byDay = new Map<number, DailyBuildCount>();
  for (const r of rows) {
    const d = toDate(r.day);
    const key = d.getTime();
    let entry = byDay.get(key);
    if (!entry) {
      entry = { day: d, ok: 0, failed: 0, stopped: 0 };
      byDay.set(key, entry);
    }
    if (r.event_type === EventType.BuildComplete) entry.ok += r.value;
    else if (r.event_type === EventType.BuildFailed) entry.failed += r.value;
    else if (r.event_type === EventType.BuildStopped) entry.stopped += r.value;
  }
  return [...byDay.values()].sort(
    (a, b) => a.day.getTime() - b.day.getTime(),
  );
}

export async function loadRecentBuilds(
  ctx: DataContext,
  limit = 50,
): Promise<BuildSummary[]> {
  // Pull terminal build events. start_time is reconstructed from end_time -
  // duration_ms (we don't have a separate start time on the end event).
  interface Row {
    time_received: Date | string;
    data?: {
      readings?: {
        build_id?: string;
        event_type?: string;
        customer_name?: string;
        customer_name_display?: string;
        duration_ms?: number;
        total_servings?: number;
        error_message?: string;
      };
    };
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, BUILD_END_TYPES) },
    { $sort: { time_received: -1 } },
    { $limit: limit },
  ]);

  return rows.map((r) => {
    const x = r.data?.readings ?? {};
    const endTime = toDate(r.time_received);
    const durationMs = x.duration_ms ?? 0;
    const startTime = new Date(endTime.getTime() - durationMs);
    return {
      buildId: x.build_id ?? "",
      customerName: x.customer_name_display ?? x.customer_name ?? "",
      customerNameKey: x.customer_name ?? "",
      startTime,
      endTime,
      durationMs,
      outcome: outcomeFromEventType(x.event_type),
      errorMessage: x.error_message ?? "",
      totalServings: x.total_servings ?? 0,
    };
  });
}

export async function loadTopCustomers(
  ctx: DataContext,
  sinceDays: number,
  limit = 10,
): Promise<CustomerStat[]> {
  interface Row {
    name_key: string | null;
    name_display: string | null;
    value: number;
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, EventType.BuildStart, sinceDays) },
    {
      $group: {
        _id: "$data.readings.customer_name",
        name_display: { $first: "$data.readings.customer_name_display" },
        value: { $sum: 1 },
      },
    },
    {
      $project: {
        _id: 0,
        name_key: "$_id",
        name_display: 1,
        value: 1,
      },
    },
    { $sort: { value: -1 } },
    { $limit: limit },
  ]);
  return rows
    .filter((r) => typeof r.name_key === "string" && r.name_key.length > 0)
    .map((r) => ({
      name: r.name_display ?? r.name_key ?? "",
      nameKey: r.name_key ?? "",
      buildCount: r.value,
    }));
}

export async function loadIngredientStats(
  ctx: DataContext,
  sinceDays: number,
): Promise<IngredientStat[]> {
  interface Row {
    name: string | null;
    attempts: number;
    successes: number;
    avg_grams_error: number | null;
  }
  // Per-ingredient grams_error & empty-bin counts come from ingredient_complete events.
  interface CompRow {
    name: string | null;
    empty_bins: number;
    avg_grams_error: number | null;
  }
  const [grabRows, compRows] = await Promise.all([
    runMQL<Row>(ctx, [
      { $match: eventMatch(ctx, EventType.GrabAttempt, sinceDays) },
      {
        $group: {
          _id: "$data.readings.ingredient_name",
          attempts: { $sum: 1 },
          successes: {
            $sum: {
              $cond: [{ $eq: ["$data.readings.outcome", "success"] }, 1, 0],
            },
          },
        },
      },
      { $project: { _id: 0, name: "$_id", attempts: 1, successes: 1 } },
    ]),
    runMQL<CompRow>(ctx, [
      { $match: eventMatch(ctx, EventType.IngredientComplete, sinceDays) },
      {
        $group: {
          _id: "$data.readings.ingredient_name",
          empty_bins: {
            $sum: {
              $cond: ["$data.readings.bin_empty_detected", 1, 0],
            },
          },
          avg_grams_error: { $avg: "$data.readings.grams_error" },
        },
      },
      { $project: { _id: 0, name: "$_id", empty_bins: 1, avg_grams_error: 1 } },
    ]),
  ]);

  const compByName = new Map(
    compRows.map((r) => [r.name ?? "", r]),
  );

  return grabRows
    .filter((r) => typeof r.name === "string" && r.name.length > 0)
    .map((r) => {
      const comp = compByName.get(r.name ?? "");
      const rate = r.attempts > 0 ? r.successes / r.attempts : 0;
      return {
        name: r.name as string,
        attempts: r.attempts,
        successes: r.successes,
        successRate: rate,
        avgGramsError: comp?.avg_grams_error ?? 0,
        emptyBinEvents: comp?.empty_bins ?? 0,
      };
    })
    .sort((a, b) => b.attempts - a.attempts);
}

// ── Queries: trend (longitudinal) ──────────────────────────────

export async function loadSuccessRateTrend(
  ctx: DataContext,
  sinceDays: number,
  bucket: BucketUnit,
): Promise<TrendPoint[]> {
  interface Row {
    bucket: Date | string;
    ok: number;
    total: number;
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, BUILD_END_TYPES, sinceDays) },
    {
      $group: {
        _id: dateTrunc(bucket),
        ok: {
          $sum: {
            $cond: [
              { $eq: ["$data.readings.event_type", EventType.BuildComplete] },
              1,
              0,
            ],
          },
        },
        total: { $sum: 1 },
      },
    },
    { $project: { _id: 0, bucket: "$_id", ok: 1, total: 1 } },
    { $sort: { bucket: 1 } },
  ]);
  return rows.map((r) => ({
    bucketStart: toDate(r.bucket),
    value: r.total > 0 ? r.ok / r.total : 0,
    sampleSize: r.total,
  }));
}

export async function loadBuildDurationTrend(
  ctx: DataContext,
  sinceDays: number,
  bucket: BucketUnit,
): Promise<DurationTrendPoint[]> {
  interface Row {
    bucket: Date | string;
    p50: number | null;
    p95: number | null;
    sample_size: number;
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, EventType.BuildComplete, sinceDays) },
    {
      $group: {
        _id: dateTrunc(bucket),
        durations: { $push: "$data.readings.duration_ms" },
        sample_size: { $sum: 1 },
      },
    },
    {
      $project: {
        _id: 0,
        bucket: "$_id",
        sample_size: 1,
        p50: {
          $arrayElemAt: [
            { $percentile: { input: "$durations", p: [0.5], method: "approximate" } },
            0,
          ],
        },
        p95: {
          $arrayElemAt: [
            { $percentile: { input: "$durations", p: [0.95], method: "approximate" } },
            0,
          ],
        },
      },
    },
    { $sort: { bucket: 1 } },
  ]);
  return rows.map((r) => ({
    bucketStart: toDate(r.bucket),
    p50: r.p50 ?? 0,
    p95: r.p95 ?? 0,
    sampleSize: r.sample_size,
  }));
}

export async function loadIngredientSuccessTrend(
  ctx: DataContext,
  sinceDays: number,
  bucket: BucketUnit,
): Promise<IngredientTrendPoint[]> {
  interface Row {
    bucket: Date | string;
    ingredient: string | null;
    successes: number;
    attempts: number;
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, EventType.GrabAttempt, sinceDays) },
    {
      $group: {
        _id: {
          bucket: dateTrunc(bucket),
          ingredient: "$data.readings.ingredient_name",
        },
        successes: {
          $sum: {
            $cond: [{ $eq: ["$data.readings.outcome", "success"] }, 1, 0],
          },
        },
        attempts: { $sum: 1 },
      },
    },
    {
      $project: {
        _id: 0,
        bucket: "$_id.bucket",
        ingredient: "$_id.ingredient",
        successes: 1,
        attempts: 1,
      },
    },
    { $sort: { bucket: 1 } },
  ]);
  return rows
    .filter((r) => typeof r.ingredient === "string" && r.ingredient.length > 0)
    .map((r) => ({
      bucketStart: toDate(r.bucket),
      ingredientName: r.ingredient as string,
      value: r.attempts > 0 ? r.successes / r.attempts : 0,
      sampleSize: r.attempts,
    }));
}

export async function loadEmptyBinTrend(
  ctx: DataContext,
  sinceDays: number,
  bucket: BucketUnit,
): Promise<IngredientTrendPoint[]> {
  interface Row {
    bucket: Date | string;
    ingredient: string | null;
    empty_bins: number;
    sample_size: number;
  }
  const rows = await runMQL<Row>(ctx, [
    { $match: eventMatch(ctx, EventType.IngredientComplete, sinceDays) },
    {
      $group: {
        _id: {
          bucket: dateTrunc(bucket),
          ingredient: "$data.readings.ingredient_name",
        },
        empty_bins: {
          $sum: {
            $cond: ["$data.readings.bin_empty_detected", 1, 0],
          },
        },
        sample_size: { $sum: 1 },
      },
    },
    {
      $project: {
        _id: 0,
        bucket: "$_id.bucket",
        ingredient: "$_id.ingredient",
        empty_bins: 1,
        sample_size: 1,
      },
    },
    { $sort: { bucket: 1 } },
  ]);
  return rows
    .filter((r) => typeof r.ingredient === "string" && r.ingredient.length > 0)
    .map((r) => ({
      bucketStart: toDate(r.bucket),
      ingredientName: r.ingredient as string,
      value: r.empty_bins,
      sampleSize: r.sample_size,
    }));
}

// ── Drill-down ─────────────────────────────────────────────────

export async function loadGrabsForBuild(
  ctx: DataContext,
  buildId: string,
): Promise<GrabAttempt[]> {
  interface Row {
    time_received: Date | string;
    data?: {
      readings?: {
        build_id?: string;
        ingredient_name?: string;
        zone_id?: number;
        attempt_index?: number;
        depth_offset_mm?: number;
        weight_change_g?: number;
        outcome?: string;
        motion_planning_failure?: boolean;
        error_message?: string;
        duration_ms?: number;
      };
    };
  }
  const rows = await runMQL<Row>(ctx, [
    {
      $match: baseMatch(ctx, {
        "data.readings.event_type": EventType.GrabAttempt,
        "data.readings.build_id": buildId,
      }),
    },
    { $sort: { time_received: 1 } },
  ]);
  return rows.map((r) => {
    const x = r.data?.readings ?? {};
    return {
      buildId: x.build_id ?? "",
      ingredientName: x.ingredient_name ?? "",
      zoneId: x.zone_id ?? -1,
      attemptIndex: x.attempt_index ?? 0,
      depthOffsetMm: x.depth_offset_mm ?? 0,
      weightChangeG: x.weight_change_g ?? 0,
      outcome: x.outcome ?? "",
      motionPlanningFailure: x.motion_planning_failure ?? false,
      errorMessage: x.error_message ?? "",
      durationMs: x.duration_ms ?? 0,
      timestamp: toDate(r.time_received),
    };
  });
}
