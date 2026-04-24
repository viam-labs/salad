<script lang="ts">
  import { MotionTools, PCD } from "@viamrobotics/motion-tools";
  import { T } from "@threlte/core";
  import * as THREE from "three";
  import type { Zone } from "../lib/types";

  interface Props {
    pcd: Uint8Array;
    zones: Zone[];
  }

  let { pcd, zones }: Props = $props();

  const ZONE_COLORS = [
    "#e74c3c",
    "#3498db",
    "#2ecc71",
    "#f39c12",
    "#9b59b6",
    "#1abc9c",
  ];

  function buildGeometry(zone: Zone): THREE.BufferGeometry {
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(zone.mesh.vertices.flatMap((v) => v));
    geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    const indices = new Uint32Array(zone.mesh.faces.flatMap((f) => f));
    geo.setIndex(new THREE.BufferAttribute(indices, 1));
    geo.computeVertexNormals();
    return geo;
  }

  const zoneGeometries = $derived(zones.map((zone) => buildGeometry(zone)));
</script>

<div class="viewer">
  <MotionTools>
    <PCD data={pcd} />
    {#each zoneGeometries as geo, i}
      <T.Mesh geometry={geo}>
        <T.MeshBasicMaterial
          color={ZONE_COLORS[i % ZONE_COLORS.length]}
          transparent={true}
          opacity={0.5}
          side={THREE.DoubleSide}
        />
      </T.Mesh>
    {/each}
  </MotionTools>
</div>

<div class="zone-legend">
  {#each zones as zone, i}
    <div class="zone-chip">
      <span class="swatch" style="background:{ZONE_COLORS[i % ZONE_COLORS.length]}"></span>
      Zone {zone.id}
    </div>
  {/each}
</div>

<style>
  .viewer {
    width: 100%;
    height: 480px;
    border-radius: 8px;
    overflow: hidden;
    background: #111;
  }

  .zone-legend {
    display: flex;
    flex-wrap: wrap;
    gap: 8px;
    margin-top: 10px;
  }

  .zone-chip {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 13px;
    color: #ccc;
  }

  .swatch {
    display: inline-block;
    width: 12px;
    height: 12px;
    border-radius: 2px;
  }
</style>
