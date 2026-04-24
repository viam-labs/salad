<script lang="ts">
  import { onMount } from "svelte";
  import { getStatus, stopBuild, getCameraStream, getSetupResult } from "../lib/robot";
  import SetupResultViewer from "./SetupResultViewer.svelte";
  import type { SetupResult } from "../lib/types";

  interface Props {
    onDone: () => void;
    onFailed: (message: string) => void;
  }

  let { onDone, onFailed }: Props = $props();

  let status = $state("Setting up station\u2026");
  let stopping = $state(false);
  let phase = $state<"running" | "review">("running");
  let setupResult = $state<SetupResult | null>(null);
  let video = $state<HTMLVideoElement | undefined>(undefined);

  onMount(() => {
    if (video) video.srcObject = getCameraStream();

    const interval = setInterval(async () => {
      try {
        const result = await getStatus();
        status = result.status ?? "";

        if (result.status === "idle" || result.status === "stopped") {
          clearInterval(interval);
          try {
            setupResult = await getSetupResult();
            phase = "review";
          } catch (err) {
            console.error("Failed to fetch setup result, proceeding anyway:", err);
            onDone();
          }
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

{#if phase === "running"}
  <div class="setup-screen">
    <h1>Setting Up Station&hellip;</h1>
    <div class="status-text">{status}</div>
    <video class="stream-video" bind:this={video} autoplay playsinline></video>
    <button class="btn-stop" disabled={stopping} onclick={handleStop}>
      {stopping ? "Stopping\u2026" : "Stop Setup"}
    </button>
  </div>
{:else}
  <div class="setup-review">
    <h1>Station Setup Complete</h1>
    {#if setupResult}
      <p class="zone-count">
        {setupResult.zones.zones.length} bin zone{setupResult.zones.zones.length !== 1 ? "s" : ""} detected
      </p>
      <SetupResultViewer pcd={setupResult.pcd} zones={setupResult.zones.zones} />
    {/if}
    <button class="btn-done" onclick={onDone}>Done</button>
  </div>
{/if}

<style>
  .setup-screen,
  .setup-review {
    display: flex;
    flex-direction: column;
    align-items: center;
    gap: 16px;
    padding: 24px;
    width: 100%;
    max-width: 900px;
    margin: 0 auto;
  }

  h1 {
    font-size: 1.6rem;
    margin: 0;
  }

  .status-text {
    color: #aaa;
    font-size: 0.95rem;
  }

  .zone-count {
    color: #aaa;
    font-size: 0.95rem;
    margin: 0;
  }

  .stream-video {
    width: 100%;
    max-width: 720px;
    border-radius: 8px;
    background: #000;
  }

  .btn-stop {
    padding: 10px 24px;
    background: #c0392b;
    color: white;
    border: none;
    border-radius: 6px;
    font-size: 1rem;
    cursor: pointer;
  }

  .btn-stop:disabled {
    opacity: 0.5;
    cursor: default;
  }

  .btn-done {
    padding: 12px 36px;
    background: #27ae60;
    color: white;
    border: none;
    border-radius: 6px;
    font-size: 1rem;
    cursor: pointer;
  }

  .btn-done:hover {
    background: #219a52;
  }
</style>
