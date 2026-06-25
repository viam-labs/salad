<script lang="ts">
  // Stacked bar chart: one group of stacked segments per x-bucket.
  // Pure SVG, scales to container via viewBox.

  interface Segment {
    label: string;
    value: number;
    color: string;
  }
  interface Group {
    label: string;
    segments: Segment[];
  }

  interface Props {
    groups: Group[];
    height?: number;
    yLabel?: string;
  }

  let { groups, height = 220, yLabel = "" }: Props = $props();

  const width = 800;
  const margin = { top: 20, right: 12, bottom: 36, left: 36 };

  let plotW = $derived(width - margin.left - margin.right);
  let plotH = $derived(height - margin.top - margin.bottom);

  let groupTotals = $derived(
    groups.map((g) => g.segments.reduce((s, x) => s + x.value, 0)),
  );

  let yMax = $derived(Math.max(1, ...groupTotals) * 1.1);
  let groupW = $derived(plotW / Math.max(groups.length, 1));
  let barW = $derived(groupW * 0.7);

  let yTicks = $derived.by(() => {
    const ticks: number[] = [];
    const n = 4;
    for (let i = 0; i <= n; i++) {
      ticks.push(Math.round((yMax * i) / n));
    }
    return ticks;
  });
</script>

<svg
  viewBox={`0 0 ${width} ${height}`}
  width="100%"
  preserveAspectRatio="xMidYMid meet"
  role="img"
  aria-label={yLabel || "bar chart"}
>
  {#each yTicks as v, i}
    {@const y = margin.top + plotH - (plotH * i) / (yTicks.length - 1)}
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
      {v}
    </text>
  {/each}

  {#each groups as g, gi}
    {@const groupX = margin.left + gi * groupW + (groupW - barW) / 2}
    {@const total = groupTotals[gi]}
    {#if total > 0}
      <g>
        {#each g.segments as seg, si}
          {@const yOffsetBelow = g.segments.slice(0, si).reduce((s, x) => s + x.value, 0)}
          {@const h = (plotH * seg.value) / yMax}
          {@const y = margin.top + plotH - (plotH * (yOffsetBelow + seg.value)) / yMax}
          {#if seg.value > 0}
            <rect
              x={groupX}
              y={y}
              width={barW}
              height={h}
              fill={seg.color}
              rx="2"
            />
          {/if}
        {/each}
        <text
          x={groupX + barW / 2}
          y={margin.top + plotH - (plotH * total) / yMax - 4}
          text-anchor="middle"
          font-size="10"
          fill="#374151"
        >
          {total}
        </text>
      </g>
    {/if}
    <text
      x={margin.left + gi * groupW + groupW / 2}
      y={margin.top + plotH + 16}
      text-anchor="middle"
      font-size="11"
      fill="#374151"
    >
      {g.label}
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
