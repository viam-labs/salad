<script lang="ts">
  import { onMount } from "svelte";
  import { buildSalad, getStatus, stopBuild, getCameraStream } from "../lib/robot";
  import { getCopy } from "../lib/theme";

  interface Props {
    order: Record<string, number>;
    customerName: string;
    onComplete: () => void;
    onStopped: () => void;
    onFailed: (message: string) => void;
  }

  let { order, customerName, onComplete, onStopped, onFailed }: Props = $props();

  const text = getCopy();

  let status = $state("Starting\u2026");
  let progress = $state(0);
  let stopping = $state(false);
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
        status = result.status ?? "";

        if (result.status === "complete") {
          clearInterval(interval);
          onComplete();
        } else if (result.status === "stopped") {
          clearInterval(interval);
          onStopped();
        } else if (result.status === "failed") {
          clearInterval(interval);
          onFailed(result.error_msg ?? "Build failed");
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
  <h1>{text.buildingTitle(customerName)}</h1>
  <div class="progress-container">
    <div class="status-text">{status}</div>
    <div class="progress-bar-bg">
      <div class="progress-bar-fill" style="width: {progress}%"></div>
    </div>
    <span class="progress-pct">{progress}%</span>
  </div>
  <video class="stream-video" bind:this={video} autoplay playsinline></video>
  <button class="btn-stop" disabled={stopping} onclick={handleStop}>
    {stopping ? "Stopping\u2026" : "Stop Build"}
  </button>
</div>
