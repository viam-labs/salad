<script lang="ts">
  import { onMount } from "svelte";
  import { buildSalad, getStatus, stopBuild, getCameraStream } from "../lib/robot";

  interface Props {
    order: Record<string, number>;
    onComplete: () => void;
    onStopped: () => void;
  }

  let { order, onComplete, onStopped }: Props = $props();

  let status = $state("Starting\u2026");
  let progress = $state(0);
  let stopping = $state(false);
  let video: HTMLVideoElement;

  onMount(() => {
    const payload: Record<string, number> = {};
    for (const [name, count] of Object.entries(order)) {
      if (count > 0) payload[name] = count;
    }
    buildSalad(payload).catch((err) =>
      console.error("build_salad error:", err),
    );

    getCameraStream("overhead-webcam")
      .then((stream) => { video.srcObject = stream; })
      .catch((err) => console.error("Camera stream error:", err));

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
  <h1>Building Your Salad&hellip;</h1>
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
