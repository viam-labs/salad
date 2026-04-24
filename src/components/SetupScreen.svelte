<script lang="ts">
  import { onMount } from "svelte";
  import { getStatus, stopBuild, getCameraStream } from "../lib/robot";

  interface Props {
    onDone: () => void;
    onFailed: (message: string) => void;
  }

  let { onDone, onFailed }: Props = $props();

  let status = $state("Setting up station\u2026");
  let stopping = $state(false);
  let video: HTMLVideoElement;

  onMount(() => {
    video.srcObject = getCameraStream();

    const interval = setInterval(async () => {
      try {
        const result = await getStatus();
        status = result.status ?? "";

        if (result.status === "idle" || result.status === "stopped") {
          clearInterval(interval);
          onDone();
        } else if (result.status === "failed") {
          clearInterval(interval);
          onFailed(result.error_msg ?? "Setup failed");
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
      console.error("Failed to stop setup:", err);
    }
  }
</script>

<div class="setup-screen">
  <h1>Setting Up Station&hellip;</h1>
  <div class="status-text">{status}</div>
  <video class="stream-video" bind:this={video} autoplay playsinline></video>
  <button class="btn-stop" disabled={stopping} onclick={handleStop}>
    {stopping ? "Stopping\u2026" : "Stop Setup"}
  </button>
</div>
