<script lang="ts">
  import { onMount } from "svelte";

  interface Props {
    onNewOrder: () => void;
  }

  let { onNewOrder }: Props = $props();

  let secondsLeft = $state(10);

  onMount(() => {
    const interval = setInterval(() => {
      secondsLeft--;
      if (secondsLeft <= 0) {
        clearInterval(interval);
        onNewOrder();
      }
    }, 1000);

    return () => clearInterval(interval);
  });
</script>

<div class="complete-screen">
  <div class="complete-emoji">&#x1F957;</div>
  <h1>Your Salad is Ready!</h1>
  <div class="complete-countdown">Next order in {secondsLeft}s</div>
  <button class="btn-new-order" onclick={onNewOrder}>New Order</button>
</div>
