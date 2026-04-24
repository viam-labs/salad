<script lang="ts">
  import { onMount } from "svelte";
  import { buildSalad, getStatus, stopBuild, getCameraStream } from "../lib/robot";
  import type { BuildStatus } from "../lib/types";

  interface Props {
    order: Record<string, number>;
    customerName: string;
    onComplete: (warnings?: string[]) => void;
    onStopped: () => void;
  }

  let { order, customerName, onComplete, onStopped }: Props = $props();

  let displayStatus = $state("Starting\u2026");
  let progress = $state(0);
  let etaSeconds = $state<number | null>(null);
  let stopping = $state(false);

  function statusLabel(result: BuildStatus): string {
    switch (result.state) {
      case "preparing": return "Preparing\u2026";
      case "adding": return result.current_ingredient ? `Adding ${result.current_ingredient}\u2026` : "Adding ingredient\u2026";
      case "delivering": return "Delivering bowl\u2026";
      case "complete": return "Complete!";
      case "stopped": return "Stopped";
      case "failed": return result.failure_reason ? `Failed: ${result.failure_reason}` : "Build failed";
      default: return result.state;
    }
  }
  let video: HTMLVideoElement;

  onMount(() => {
    const payload: Record<string, number> = {};
    for (const [name, count] of Object.entries(order)) {
      if (count > 0) payload[name] = count;
    }
    buildSalad(payload, customerName).catch((err) =>
      console.error("build_salad error:", err),
    );

    video.srcObject = getCameraStream();

    const interval = setInterval(async () => {
      try {
        const result = await getStatus();
        progress = Math.round(result.progress ?? 0);
        displayStatus = statusLabel(result);
        etaSeconds = result.eta_seconds ?? null;

        if (result.state === "complete") {
          clearInterval(interval);
          onComplete(result.warnings);
        } else if (result.state === "stopped" || result.state === "failed") {
          clearInterval(interval);
          onStopped();
        }
      } catch (err) {
        console.error("Status poll error:", err);
      }
    }, 1000);

    return () => clearInterval(interval);
  });

  async function handleStop() {
    stopping = true;
    try {
      await stopBuild();
    } catch (err) {
      console.error("Failed to stop build:", err);
    }
  }
</script>

<div class="building-screen">
  <h1>{customerName ? `Building ${customerName}'s Salad\u2026` : "Building Your Salad\u2026"}</h1>
  <div class="progress-container">
    <div class="status-text">{displayStatus}</div>
    <div class="progress-bar-bg">
      <div class="progress-bar-fill" style="width: {progress}%"></div>
    </div>
    <span class="progress-pct">{progress}%</span>
    {#if etaSeconds !== null && etaSeconds > 0}
      <div class="eta">~{Math.round(etaSeconds)}s remaining</div>
    {/if}
  </div>
  <video class="stream-video" bind:this={video} autoplay playsinline></video>
  <button class="btn-stop" disabled={stopping} onclick={handleStop}>
    {stopping ? "Stopping\u2026" : "Stop Build"}
  </button>
</div>
