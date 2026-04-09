<script lang="ts">
  import { onMount } from "svelte";
  import { getQueue, cancelOrder } from "../lib/robot";
  import type { QueueOrder } from "../lib/types";

  interface Props {
    orderId: string | null;
    displayOnly?: boolean;
    onBuilding?: (customerName: string) => void;
    onComplete?: () => void;
  }

  let { orderId, displayOnly = false, onBuilding, onComplete }: Props = $props();

  let orders: QueueOrder[] = $state([]);
  let avgBuildSec = $state(0);
  let cancelling = $state(false);

  function formatWait(seconds: number | undefined): string {
    if (!seconds || seconds <= 0) return "";
    const mins = Math.ceil(seconds / 60);
    return `~${mins} min`;
  }

  onMount(() => {
    let cancelled = false;

    const poll = async () => {
      try {
        const snapshot = await getQueue();
        if (cancelled) return;
        orders = snapshot.orders ?? [];
        avgBuildSec = snapshot.avg_build_duration_sec ?? 0;

        // If this user's order is now building, transition to building screen.
        if (!displayOnly && orderId) {
          const myOrder = orders.find((o) => o.id === orderId);
          if (myOrder?.status === "building" && onBuilding) {
            onBuilding(myOrder.customer_name);
          }
          // If our order is no longer in the queue (completed or gone), go to complete.
          if (!myOrder && onComplete) {
            onComplete();
          }
        }
      } catch (err) {
        console.error("Queue poll error:", err);
      }
    };

    poll();
    const interval = setInterval(poll, 2000);
    return () => {
      cancelled = true;
      clearInterval(interval);
    };
  });

  async function handleCancel() {
    if (!orderId) return;
    cancelling = true;
    try {
      await cancelOrder(orderId);
    } catch (err) {
      console.error("Failed to cancel order:", err);
    }
    cancelling = false;
  }
</script>

<div class="queue-board">
  <h1>Order Board</h1>

  {#if orders.length === 0}
    <div class="queue-empty">No orders in queue</div>
  {:else}
    <div class="queue-list">
      {#each orders as order, i (order.id)}
        <div
          class="queue-item"
          class:queue-item--building={order.status === "building"}
          class:queue-item--mine={order.id === orderId}
        >
          <div class="queue-item-position">
            {#if order.status === "building"}
              &#x1F373;
            {:else}
              #{order.position ?? i + 1}
            {/if}
          </div>
          <div class="queue-item-info">
            <div class="queue-item-name">
              {order.customer_name || "Guest"}
              {#if order.id === orderId}
                <span class="queue-item-yours">(Your Order)</span>
              {/if}
            </div>
            <div class="queue-item-status">
              {#if order.status === "building"}
                <span class="badge badge--building">Building</span>
                <span class="queue-item-step">{order.current_step}</span>
              {:else}
                <span class="badge badge--queued">Queued</span>
                <span class="queue-item-wait">{formatWait(order.estimated_wait_sec)}</span>
              {/if}
            </div>
            {#if order.status === "building" && order.progress != null}
              <div class="queue-item-progress-bg">
                <div class="queue-item-progress-fill" style="width: {Math.round(order.progress)}%"></div>
              </div>
            {/if}
          </div>
          {#if !displayOnly && order.id === orderId && order.status === "queued"}
            <button class="btn-cancel" disabled={cancelling} onclick={handleCancel}>
              {cancelling ? "Cancelling\u2026" : "Cancel"}
            </button>
          {/if}
        </div>
      {/each}
    </div>
  {/if}
</div>
