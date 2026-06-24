<script lang="ts">
  import { onMount, onDestroy } from "svelte";
  import { getDashboardContext } from "../lib/robot";
  import {
    loadDailyBuildCounts,
    loadRecentBuilds,
    loadTopCustomers,
    loadIngredientStats,
    loadSuccessRateTrend,
    loadBuildDurationTrend,
    loadIngredientSuccessTrend,
    loadEmptyBinTrend,
    formatDay,
    type BuildSummary,
    type CustomerStat,
    type DailyBuildCount,
    type DataContext,
    type IngredientStat,
    type TrendPoint,
    type DurationTrendPoint,
    type IngredientTrendPoint,
    type BucketUnit,
  } from "../lib/data";
  import BarChart from "./charts/BarChart.svelte";
  import LineChart from "./charts/LineChart.svelte";
  import StatTable from "./charts/StatTable.svelte";

  interface Props {
    onBack: () => void;
  }
  let { onBack }: Props = $props();

  // ── Time-range selector ──────────────────────────────
  type RangeKey = "30d" | "90d" | "6mo" | "all";
  const RANGES: { key: RangeKey; label: string; days: number; bucket: BucketUnit }[] = [
    { key: "30d", label: "30d", days: 30, bucket: "day" },
    { key: "90d", label: "90d", days: 90, bucket: "week" },
    { key: "6mo", label: "6mo", days: 180, bucket: "week" },
    { key: "all", label: "all", days: 365 * 5, bucket: "week" },
  ];
  let range = $state<RangeKey>("90d");
  let activeRange = $derived(RANGES.find((r) => r.key === range)!);

  // ── State ───────────────────────────────────────────
  let loading = $state(true);
  let error = $state("");

  let dailyCounts = $state<DailyBuildCount[]>([]);
  let recentBuilds = $state<BuildSummary[]>([]);
  let topCustomers = $state<CustomerStat[]>([]);
  let ingredientStats = $state<IngredientStat[]>([]);

  let successTrend = $state<TrendPoint[]>([]);
  let durationTrend = $state<DurationTrendPoint[]>([]);
  let ingredientSuccessTrend = $state<IngredientTrendPoint[]>([]);
  let emptyBinTrend = $state<IngredientTrendPoint[]>([]);

  // ── Derived UI shapes ───────────────────────────────
  let summaryStats = $derived.by(() => {
    const totalBuilds = recentBuilds.length;
    const ok = recentBuilds.filter((b) => b.outcome === "ok").length;
    const successPct = totalBuilds > 0 ? Math.round((ok / totalBuilds) * 100) : 0;
    const avgMs =
      totalBuilds > 0
        ? recentBuilds.reduce((s, b) => s + b.durationMs, 0) / totalBuilds
        : 0;
    return { totalBuilds, successPct, avgMs };
  });

  let dailyGroups = $derived(
    dailyCounts.map((d) => ({
      label: formatDay(d.day),
      segments: [
        { label: "ok", value: d.ok, color: "#86efac" },
        { label: "stopped", value: d.stopped, color: "#fbbf24" },
        { label: "failed", value: d.failed, color: "#fca5a5" },
      ],
    })),
  );

  let durationSeries = $derived.by(() => {
    if (durationTrend.length === 0) return [];
    const p50 = durationTrend.map((d) => ({
      x: d.bucketStart,
      y: d.p50 / 1000,
      sampleSize: d.sampleSize,
    }));
    const p95 = durationTrend.map((d) => ({
      x: d.bucketStart,
      y: d.p95 / 1000,
      sampleSize: d.sampleSize,
    }));
    return [
      { name: "p50 (s)", color: "#3b82f6", points: p50, upperPoints: p95 },
      { name: "p95 (s)", color: "#1e40af", points: p95 },
    ];
  });

  let successSeries = $derived(
    successTrend.length === 0
      ? []
      : [
          {
            name: "success rate",
            color: "#16a34a",
            points: successTrend.map((p) => ({
              x: p.bucketStart,
              y: p.value,
              sampleSize: p.sampleSize,
            })),
          },
        ],
  );

  // Color picker for per-ingredient lines: stable per name hash.
  function ingredientColor(name: string): string {
    let h = 0;
    for (let i = 0; i < name.length; i++) h = (h * 31 + name.charCodeAt(i)) >>> 0;
    return `hsl(${h % 360}, 65%, 50%)`;
  }

  function groupByIngredient(points: IngredientTrendPoint[]) {
    const map = new Map<string, IngredientTrendPoint[]>();
    for (const p of points) {
      const arr = map.get(p.ingredientName) ?? [];
      arr.push(p);
      map.set(p.ingredientName, arr);
    }
    return [...map.entries()].map(([name, pts]) => ({
      name,
      color: ingredientColor(name),
      points: pts.map((p) => ({
        x: p.bucketStart,
        y: p.value,
        sampleSize: p.sampleSize,
      })),
    }));
  }

  let ingredientSuccessSeries = $derived(
    groupByIngredient(ingredientSuccessTrend),
  );
  let emptyBinSeries = $derived(groupByIngredient(emptyBinTrend));

  // ── Formatters ──────────────────────────────────────
  function formatDuration(ms: number): string {
    const totalSec = Math.round(ms / 1000);
    const m = Math.floor(totalSec / 60);
    const s = totalSec % 60;
    if (m === 0) return `${s}s`;
    return `${m}m${s.toString().padStart(2, "0")}s`;
  }
  function formatTime(d: Date): string {
    return d.toLocaleTimeString(undefined, { hour: "2-digit", minute: "2-digit" });
  }
  function formatPct(v: number): string {
    return `${Math.round(v * 100)}%`;
  }

  // ── Loading & refresh ──────────────────────────────
  let timer: ReturnType<typeof setInterval> | null = null;

  let ctx: DataContext | null = null;

  async function refreshAll() {
    const sinceDays = activeRange.days;
    const bucket = activeRange.bucket;
    try {
      if (!ctx) ctx = await getDashboardContext();
      const [
        d,
        r,
        c,
        ing,
        st,
        dt,
        ist,
        ebt,
      ] = await Promise.all([
        loadDailyBuildCounts(ctx, 7),
        loadRecentBuilds(ctx, 50),
        loadTopCustomers(ctx, 7, 5),
        loadIngredientStats(ctx, 7),
        loadSuccessRateTrend(ctx, sinceDays, bucket),
        loadBuildDurationTrend(ctx, sinceDays, bucket),
        loadIngredientSuccessTrend(ctx, sinceDays, bucket),
        loadEmptyBinTrend(ctx, sinceDays, bucket),
      ]);
      dailyCounts = d;
      recentBuilds = r;
      topCustomers = c;
      ingredientStats = ing;
      successTrend = st;
      durationTrend = dt;
      ingredientSuccessTrend = ist;
      emptyBinTrend = ebt;
      error = "";
    } catch (err) {
      error = err instanceof Error ? err.message : String(err);
      console.error("dashboard refresh failed:", err);
    } finally {
      loading = false;
    }
  }

  // Re-fetch when the range changes (covers initial load).
  $effect(() => {
    void range;
    loading = true;
    refreshAll();
  });

  onMount(() => {
    timer = setInterval(refreshAll, 30_000);
  });
  onDestroy(() => {
    if (timer) clearInterval(timer);
  });

  // Recent-builds table columns + ingredient table columns
  let recentColumns = $derived([
    {
      key: "time",
      label: "Time",
      sortValue: (r: BuildSummary) => r.endTime.getTime(),
      render: (r: BuildSummary) =>
        `${formatDay(r.endTime)} ${formatTime(r.endTime)}`,
    },
    {
      key: "customer",
      label: "Customer",
      sortValue: (r: BuildSummary) => r.customerName.toLowerCase(),
      render: (r: BuildSummary) => r.customerName || "—",
    },
    {
      key: "servings",
      label: "Servings",
      align: "right" as const,
      sortValue: (r: BuildSummary) => r.totalServings,
      render: (r: BuildSummary) => String(r.totalServings),
    },
    {
      key: "duration",
      label: "Duration",
      align: "right" as const,
      sortValue: (r: BuildSummary) => r.durationMs,
      render: (r: BuildSummary) => formatDuration(r.durationMs),
    },
    {
      key: "outcome",
      label: "Outcome",
      sortValue: (r: BuildSummary) => r.outcome,
      render: (r: BuildSummary) => r.outcome,
    },
  ]);

  let ingredientColumns = $derived([
    {
      key: "name",
      label: "Ingredient",
      sortValue: (r: IngredientStat) => r.name,
      render: (r: IngredientStat) => r.name,
    },
    {
      key: "attempts",
      label: "Attempts",
      align: "right" as const,
      sortValue: (r: IngredientStat) => r.attempts,
      render: (r: IngredientStat) => String(r.attempts),
    },
    {
      key: "successRate",
      label: "Success",
      align: "right" as const,
      sortValue: (r: IngredientStat) => r.successRate,
      render: (r: IngredientStat) => formatPct(r.successRate),
    },
    {
      key: "avgGramsError",
      label: "Avg g error",
      align: "right" as const,
      sortValue: (r: IngredientStat) => r.avgGramsError,
      render: (r: IngredientStat) =>
        `${r.avgGramsError >= 0 ? "+" : ""}${r.avgGramsError.toFixed(1)}g`,
    },
    {
      key: "emptyBinEvents",
      label: "Empty bins",
      align: "right" as const,
      sortValue: (r: IngredientStat) => r.emptyBinEvents,
      render: (r: IngredientStat) => String(r.emptyBinEvents),
    },
  ]);
</script>

<div class="dashboard">
  <header class="dashboard-header">
    <h1>Salad Dashboard</h1>
    <div class="header-controls">
      <div class="range-selector" role="tablist" aria-label="Time range">
        {#each RANGES as r}
          <button
            type="button"
            class:active={range === r.key}
            onclick={() => (range = r.key)}
            role="tab"
            aria-selected={range === r.key}
          >
            {r.label}
          </button>
        {/each}
      </div>
      <button class="back-btn" onclick={onBack} aria-label="Back to ordering">
        ← Back
      </button>
    </div>
  </header>

  {#if error}
    <div class="dashboard-error">Failed to load: {error}</div>
  {/if}

  {#if loading && recentBuilds.length === 0}
    <div class="dashboard-loading">Loading dashboard…</div>
  {:else}
    <section class="dashboard-section">
      <h2>Trends</h2>
      <p class="section-sub">
        Longitudinal health · {activeRange.label} · bucketed by {activeRange.bucket}.
        Hollow points mean small sample size.
      </p>
      <div class="chart-grid">
        <div class="chart-card">
          <h3>Build success rate</h3>
          <LineChart
            series={successSeries}
            yMin={0}
            yMax={1}
            yFormat={(v) => `${Math.round(v * 100)}%`}
            legend={false}
          />
        </div>
        <div class="chart-card">
          <h3>Build duration (seconds)</h3>
          <LineChart
            series={durationSeries}
            yMin={0}
            yFormat={(v) => `${Math.round(v)}s`}
          />
        </div>
        <div class="chart-card">
          <h3>Ingredient success rate</h3>
          <LineChart
            series={ingredientSuccessSeries}
            yMin={0}
            yMax={1}
            yFormat={(v) => `${Math.round(v * 100)}%`}
          />
        </div>
        <div class="chart-card">
          <h3>Empty-bin events</h3>
          <LineChart
            series={emptyBinSeries}
            yMin={0}
            yFormat={(v) => String(Math.round(v))}
          />
        </div>
      </div>
    </section>

    <section class="dashboard-section">
      <h2>Current window (last 7 days)</h2>
      <div class="summary-cards">
        <div class="summary-card">
          <div class="summary-value">{summaryStats.totalBuilds}</div>
          <div class="summary-label">builds</div>
        </div>
        <div class="summary-card">
          <div class="summary-value">{summaryStats.successPct}%</div>
          <div class="summary-label">success rate</div>
        </div>
        <div class="summary-card">
          <div class="summary-value">{formatDuration(summaryStats.avgMs)}</div>
          <div class="summary-label">avg duration</div>
        </div>
      </div>

      <div class="chart-grid">
        <div class="chart-card">
          <h3>Daily builds</h3>
          <BarChart groups={dailyGroups} />
        </div>
        <div class="chart-card">
          <h3>Top customers</h3>
          {#if topCustomers.length === 0}
            <div class="empty">No customers in the window.</div>
          {:else}
            <ol class="leaderboard">
              {#each topCustomers as c, i}
                <li>
                  <span class="rank">{i + 1}.</span>
                  <span class="name">{c.name}</span>
                  <span class="count">{c.buildCount}</span>
                </li>
              {/each}
            </ol>
          {/if}
        </div>
      </div>
    </section>

    <section class="dashboard-section">
      <h2>Ingredient performance</h2>
      <StatTable
        rows={ingredientStats}
        columns={ingredientColumns}
        emptyMessage="No grab data in the window."
        initialSortKey="attempts"
      />
    </section>

    <section class="dashboard-section">
      <h2>Recent builds</h2>
      <StatTable
        rows={recentBuilds}
        columns={recentColumns}
        emptyMessage="No builds yet."
        initialSortKey="time"
      />
    </section>
  {/if}
</div>

<style>
  .dashboard {
    padding: 1rem 1.25rem 4rem;
    max-width: 1200px;
    margin: 0 auto;
    font-family: var(--font-body, system-ui), sans-serif;
    color: #111827;
  }

  .dashboard-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    flex-wrap: wrap;
    gap: 0.75rem;
    margin-bottom: 1.25rem;
  }
  .dashboard-header h1 {
    margin: 0;
    font-size: 1.5rem;
    font-family: var(--font-display, system-ui);
  }
  .header-controls {
    display: flex;
    align-items: center;
    gap: 0.75rem;
    flex-wrap: wrap;
  }

  .range-selector {
    display: inline-flex;
    background: #f3f4f6;
    border-radius: 0.5rem;
    padding: 0.15rem;
  }
  .range-selector button {
    border: none;
    background: transparent;
    padding: 0.45rem 0.85rem;
    font-size: 0.9rem;
    color: #374151;
    border-radius: 0.4rem;
    cursor: pointer;
    min-height: 44px;
  }
  .range-selector button.active {
    background: white;
    color: #111827;
    box-shadow: 0 1px 2px rgba(0, 0, 0, 0.08);
  }

  .back-btn {
    background: white;
    border: 1px solid #d1d5db;
    border-radius: 0.5rem;
    padding: 0.5rem 0.9rem;
    font-size: 0.95rem;
    cursor: pointer;
    min-height: 44px;
  }
  .back-btn:hover {
    background: #f9fafb;
  }

  .dashboard-error {
    background: #fef2f2;
    color: #991b1b;
    padding: 0.75rem 1rem;
    border-radius: 0.5rem;
    margin-bottom: 1rem;
  }
  .dashboard-loading {
    padding: 3rem 1rem;
    text-align: center;
    color: #6b7280;
  }

  .dashboard-section {
    margin-bottom: 2rem;
  }
  .dashboard-section h2 {
    font-size: 1.1rem;
    margin: 0 0 0.5rem;
    color: #111827;
  }
  .section-sub {
    margin: 0 0 1rem;
    font-size: 0.85rem;
    color: #6b7280;
  }

  .summary-cards {
    display: flex;
    flex-wrap: wrap;
    gap: 0.75rem;
    margin-bottom: 1rem;
  }
  .summary-card {
    flex: 1 1 9rem;
    min-width: 9rem;
    background: white;
    border: 1px solid #e5e7eb;
    border-radius: 0.6rem;
    padding: 0.9rem 1rem;
  }
  .summary-value {
    font-size: 1.5rem;
    font-weight: 600;
    color: #111827;
  }
  .summary-label {
    font-size: 0.85rem;
    color: #6b7280;
    margin-top: 0.15rem;
  }

  .chart-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
    gap: 1rem;
  }
  .chart-card {
    background: white;
    border: 1px solid #e5e7eb;
    border-radius: 0.6rem;
    padding: 0.85rem 1rem;
  }
  .chart-card h3 {
    margin: 0 0 0.5rem;
    font-size: 0.95rem;
    color: #111827;
  }

  .leaderboard {
    list-style: none;
    padding: 0;
    margin: 0;
  }
  .leaderboard li {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    padding: 0.35rem 0;
    font-size: 0.95rem;
  }
  .leaderboard .rank {
    color: #6b7280;
    min-width: 1.5rem;
  }
  .leaderboard .name {
    flex: 1;
  }
  .leaderboard .count {
    font-weight: 600;
  }

  .empty {
    color: #6b7280;
    padding: 1rem 0;
    font-size: 0.9rem;
  }

  @media (max-width: 600px) {
    .dashboard {
      padding: 0.75rem 0.75rem 4rem;
    }
    .chart-grid {
      grid-template-columns: 1fr;
    }
    .summary-card {
      flex: 1 1 100%;
    }
  }
</style>
