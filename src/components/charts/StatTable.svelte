<script lang="ts" generics="T">
  // Sortable stat table. On mobile, switches to a stacked card list.
  // Caller supplies columns with render functions so cell content stays typed.

  interface Column {
    key: string;
    label: string;
    align?: "left" | "right";
    sortValue?: (row: T) => number | string;
    render: (row: T) => string;
  }

  interface Props {
    rows: T[];
    columns: Column[];
    onRowClick?: (row: T) => void;
    emptyMessage?: string;
    initialSortKey?: string;
    initialSortDir?: "asc" | "desc";
  }

  let {
    rows,
    columns,
    onRowClick,
    emptyMessage = "No data",
    initialSortKey,
    initialSortDir = "desc",
  }: Props = $props();

  let sortKey = $state(initialSortKey ?? columns[0]?.key ?? "");
  let sortDir = $state<"asc" | "desc">(initialSortDir);

  let sortedRows = $derived.by(() => {
    const col = columns.find((c) => c.key === sortKey);
    if (!col || !col.sortValue) return rows;
    const sv = col.sortValue;
    const dir = sortDir === "asc" ? 1 : -1;
    return [...rows].sort((a, b) => {
      const av = sv(a);
      const bv = sv(b);
      if (av < bv) return -1 * dir;
      if (av > bv) return 1 * dir;
      return 0;
    });
  });

  function toggleSort(key: string) {
    if (sortKey === key) {
      sortDir = sortDir === "asc" ? "desc" : "asc";
    } else {
      sortKey = key;
      sortDir = "desc";
    }
  }
</script>

{#if rows.length === 0}
  <div class="stat-table-empty">{emptyMessage}</div>
{:else}
  <table class="stat-table">
    <thead>
      <tr>
        {#each columns as col}
          <th
            class:right={col.align === "right"}
            class:sortable={!!col.sortValue}
            onclick={() => col.sortValue && toggleSort(col.key)}
          >
            {col.label}
            {#if col.sortValue && sortKey === col.key}
              <span class="sort-indicator">
                {sortDir === "asc" ? "▲" : "▼"}
              </span>
            {/if}
          </th>
        {/each}
      </tr>
    </thead>
    <tbody>
      {#each sortedRows as row}
        <tr
          class:clickable={!!onRowClick}
          onclick={() => onRowClick?.(row)}
        >
          {#each columns as col}
            <td class:right={col.align === "right"}>{col.render(row)}</td>
          {/each}
        </tr>
      {/each}
    </tbody>
  </table>

  <ul class="stat-cards">
    {#each sortedRows as row}
      <li
        class="stat-card"
        class:clickable={!!onRowClick}
        onclick={() => onRowClick?.(row)}
        role={onRowClick ? "button" : undefined}
        tabindex={onRowClick ? 0 : undefined}
        onkeydown={(e) => {
          if (onRowClick && (e.key === "Enter" || e.key === " ")) {
            e.preventDefault();
            onRowClick(row);
          }
        }}
      >
        {#each columns as col}
          <div class="stat-card-row">
            <span class="stat-card-label">{col.label}</span>
            <span class="stat-card-value">{col.render(row)}</span>
          </div>
        {/each}
      </li>
    {/each}
  </ul>
{/if}

<style>
  .stat-table {
    width: 100%;
    border-collapse: collapse;
    font-size: 0.95rem;
  }
  .stat-table th,
  .stat-table td {
    padding: 0.5rem 0.75rem;
    text-align: left;
    border-bottom: 1px solid #e5e7eb;
  }
  .stat-table th {
    font-weight: 600;
    color: #374151;
    background: #f9fafb;
    user-select: none;
  }
  .stat-table th.sortable {
    cursor: pointer;
  }
  .stat-table th.right,
  .stat-table td.right {
    text-align: right;
  }
  .stat-table tr.clickable {
    cursor: pointer;
  }
  .stat-table tr.clickable:hover {
    background: #f3f4f6;
  }
  .sort-indicator {
    font-size: 0.7em;
    margin-left: 0.25rem;
  }
  .stat-table-empty {
    padding: 1.5rem;
    text-align: center;
    color: #6b7280;
  }
  .stat-cards {
    display: none;
    list-style: none;
    padding: 0;
    margin: 0;
  }
  .stat-card {
    background: #f9fafb;
    border-radius: 0.5rem;
    padding: 0.75rem 1rem;
    margin-bottom: 0.5rem;
  }
  .stat-card.clickable {
    cursor: pointer;
  }
  .stat-card-row {
    display: flex;
    justify-content: space-between;
    padding: 0.2rem 0;
    font-size: 0.9rem;
  }
  .stat-card-label {
    color: #6b7280;
  }
  .stat-card-value {
    color: #111827;
    font-weight: 500;
  }

  @media (max-width: 600px) {
    .stat-table {
      display: none;
    }
    .stat-cards {
      display: block;
    }
  }
</style>
