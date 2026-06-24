<script lang="ts">
  // Multi-series line chart with optional p50/p95 shaded band.
  // Bucket points with sampleSize < smallSampleThreshold render hollow so a
  // single bad day doesn't look like a regression.

  interface Point {
    x: Date;
    y: number;
    sampleSize?: number;
  }
  interface Series {
    name: string;
    color: string;
    points: Point[];
    // Optional upper band (e.g. p95). When set, the area between points and
    // upperPoints is shaded with the series color at low opacity.
    upperPoints?: Point[];
  }

  interface Props {
    series: Series[];
    height?: number;
    yMin?: number;
    yMax?: number;
    yFormat?: (v: number) => string;
    smallSampleThreshold?: number;
    legend?: boolean;
  }

  let {
    series,
    height = 220,
    yMin,
    yMax,
    yFormat = (v: number) => String(Math.round(v * 100) / 100),
    smallSampleThreshold = 5,
    legend = true,
  }: Props = $props();

  const width = 800;
  const margin = { top: 16, right: 16, bottom: 36, left: 44 };

  let plotW = $derived(width - margin.left - margin.right);
  let plotH = $derived(height - margin.top - margin.bottom);

  let allPoints = $derived(
    series.flatMap((s) => [...s.points, ...(s.upperPoints ?? [])]),
  );
  let xs = $derived(allPoints.map((p) => p.x.getTime()));
  let ys = $derived(allPoints.map((p) => p.y));

  let xMinV = $derived(xs.length ? Math.min(...xs) : 0);
  let xMaxV = $derived(xs.length ? Math.max(...xs) : 1);
  let yMinV = $derived(yMin ?? (ys.length ? Math.min(...ys, 0) : 0));
  let yMaxV = $derived(yMax ?? (ys.length ? Math.max(...ys) * 1.1 : 1));

  function xScale(t: number): number {
    if (xMaxV === xMinV) return margin.left + plotW / 2;
    return margin.left + ((t - xMinV) / (xMaxV - xMinV)) * plotW;
  }
  function yScale(v: number): number {
    if (yMaxV === yMinV) return margin.top + plotH / 2;
    return margin.top + plotH - ((v - yMinV) / (yMaxV - yMinV)) * plotH;
  }

  function linePath(points: Point[]): string {
    if (points.length === 0) return "";
    return points
      .map(
        (p, i) =>
          `${i === 0 ? "M" : "L"}${xScale(p.x.getTime())},${yScale(p.y)}`,
      )
      .join(" ");
  }

  function bandPath(lower: Point[], upper: Point[]): string {
    if (lower.length === 0 || upper.length === 0) return "";
    const top = upper.map((p) => `${xScale(p.x.getTime())},${yScale(p.y)}`);
    const bot = [...lower]
      .reverse()
      .map((p) => `${xScale(p.x.getTime())},${yScale(p.y)}`);
    return `M${top.join(" L")} L${bot.join(" L")} Z`;
  }

  let yTicks = $derived.by(() => {
    const n = 4;
    const out: number[] = [];
    for (let i = 0; i <= n; i++) {
      out.push(yMinV + ((yMaxV - yMinV) * i) / n);
    }
    return out;
  });

  let xTicks = $derived.by(() => {
    if (allPoints.length === 0) return [] as { t: number; label: string }[];
    const sorted = [...new Set(xs)].sort((a, b) => a - b);
    const max = 6;
    const step = Math.max(1, Math.ceil(sorted.length / max));
    return sorted
      .filter((_, i) => i % step === 0)
      .map((t) => {
        const d = new Date(t);
        return {
          t,
          label: d.toLocaleDateString(undefined, {
            month: "short",
            day: "numeric",
          }),
        };
      });
  });
</script>

<svg
  viewBox={`0 0 ${width} ${height}`}
  width="100%"
  preserveAspectRatio="xMidYMid meet"
  role="img"
>
  {#each yTicks as v, i}
    {@const y = yScale(v)}
    <line
      x1={margin.left}
      x2={margin.left + plotW}
      y1={y}
      y2={y}
      stroke="#e5e7eb"
      stroke-width="1"
    />
    <text
      x={margin.left - 6}
      y={y}
      text-anchor="end"
      dominant-baseline="middle"
      font-size="11"
      fill="#6b7280"
    >
      {yFormat(v)}
    </text>
  {/each}

  {#each series as s}
    {#if s.upperPoints && s.upperPoints.length === s.points.length}
      <path d={bandPath(s.points, s.upperPoints)} fill={s.color} opacity="0.15" />
    {/if}
    <path d={linePath(s.points)} fill="none" stroke={s.color} stroke-width="2" />
    {#each s.points as p}
      {@const isSmall =
        typeof p.sampleSize === "number" && p.sampleSize < smallSampleThreshold}
      <circle
        cx={xScale(p.x.getTime())}
        cy={yScale(p.y)}
        r="3.5"
        fill={isSmall ? "white" : s.color}
        stroke={s.color}
        stroke-width="1.5"
      >
        <title
          >{p.x.toLocaleDateString()} · {yFormat(p.y)}{p.sampleSize !== undefined
            ? ` · n=${p.sampleSize}`
            : ""}</title
        >
      </circle>
    {/each}
  {/each}

  {#each xTicks as tick}
    <text
      x={xScale(tick.t)}
      y={margin.top + plotH + 16}
      text-anchor="middle"
      font-size="11"
      fill="#374151"
    >
      {tick.label}
    </text>
  {/each}

  <line
    x1={margin.left}
    x2={margin.left + plotW}
    y1={margin.top + plotH}
    y2={margin.top + plotH}
    stroke="#9ca3af"
    stroke-width="1"
  />
</svg>

{#if legend && series.length > 1}
  <div class="chart-legend">
    {#each series as s}
      <span class="chart-legend-item">
        <span class="chart-legend-swatch" style:background={s.color}></span>
        {s.name}
      </span>
    {/each}
  </div>
{/if}

<style>
  .chart-legend {
    display: flex;
    flex-wrap: wrap;
    gap: 0.75rem;
    margin-top: 0.5rem;
    font-size: 0.85rem;
    color: #374151;
  }
  .chart-legend-item {
    display: inline-flex;
    align-items: center;
    gap: 0.35rem;
  }
  .chart-legend-swatch {
    display: inline-block;
    width: 10px;
    height: 10px;
    border-radius: 2px;
  }
</style>
