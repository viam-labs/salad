# Module salad

A Viam module for automated salad assembly. The module pairs a custom
[Svelte application](#application) with a stack of generic services and
vision/sensor components that, together, drive two arms to pick
ingredients from segmented bins, deliver a bowl, pour dressings, and
report safety status.

## Models

Per-model documentation lives in its own file alongside this README.
Each file is named `<namespace>_<module>_<model>.md`.

| Model | API | Description |
|---|---|---|
| [`ncs:salad:build-coordinator`](ncs_salad_build-coordinator.md) | `rdk:service:generic` | Top-level orchestrator. Drives the full salad assembly flow: reads the scale, grabs ingredients to target weights, delivers the bowl, pours dressings, and runs `chefs_kiss`. Also implements `setup_station`, which captures and segments the bin mesh used by `grabber-controls`. |
| [`ncs:salad:grabber-controls`](ncs_salad_grabber-controls.md) | `rdk:service:generic` | Plans and executes a single ingredient grab from a segmented bin and drops it into the bowl. Uses the `motion` service with the bin mesh as a world obstacle, plus saved-position switches for the bowl drop. |
| [`ncs:salad:bowl-controls`](ncs_salad_bowl-controls.md) | `rdk:service:generic` | Drives the right arm + gripper for bowl pickup and delivery, plus an optional "lil-arm" for grabbing bowls/lids from a stack using a force-controlled descent. |
| [`ncs:salad:dressing-controls`](ncs_salad_dressing-controls.md) | `rdk:service:generic` | Picks up a dressing bottle, runs a multi-pose pour sequence with optional shakes, and returns the bottle to its slot. Supports multiple bottles via a `dressings` map. |
| [`ncs:salad:chefs-kiss-controls`](ncs_salad_chefs-kiss-controls.md) | `rdk:service:generic` | Performs a short "chef's kiss" seasoning gesture over the bowl at the end of every build. |
| [`ncs:salad:maintenance-sensor`](ncs_salad_maintenance-sensor.md) | `rdk:component:sensor` | Reports `is_safe` and `build_status` based on the build coordinator's status. Wire this into a Viam maintenance window so reconfigs and restarts don't interrupt an in-flight salad. |
| [`ncs:salad:passthrough-to-camera`](ncs_salad_passthrough-to-camera.md) | `rdk:service:vision` | Vision service that forwards `NextPointCloud` from a configured camera through the vision API. |
| [`ncs:salad:file-vision`](ncs_salad_file-vision.md) | `rdk:service:vision` | Vision service that returns a fixed 3D mesh loaded from a `.ply` file. |
| [`ncs:salad:supply-detector`](ncs_salad_supply-detector.md) | `rdk:service:generic` | Estimates per-bin ingredient supply (`available` / `low` / `unknown`) from rectangular ROIs on an overhead camera frame. |

## Application

`meta.json` ships a single-machine application named `salad` whose
entrypoint is `dist/index.html`. The front-end is built with Svelte
(see `src/`, `svelte.config.js`, `vite.config.js`) and is used to drive
the build coordinator's commands and visualize captured PCDs and the
segmented bin mesh.

## CLI

`./bin/salad-cli` is a companion CLI that exposes the filter and meshifier
sub-commands used at runtime, so you can tune `build-coordinator.filter`
against a captured PCD before changing the live config. Example:

```bash
./bin/salad-cli filter --input setup-20251010-101010.pcd --viz \
  --voxel 10 --neighbor-radius 1 --min-neighbors 8 --min-component-voxels 1000
```

## Setup workflow

Before the first build, the station must be set up so that
`grabber-controls` has a segmented bin mesh to plan against:

1. Configure all of the models above on a single machine, with the
   correct cross-references (`build-coordinator` depends on
   `grabber-controls`, `bowl-controls`, `dressing-controls`,
   `chefs-kiss-controls`, the scale sensor, and optionally the overhead
   camera and a text-to-speech service).
2. Place the ingredient bins under the imaging camera.
3. Call `build-coordinator.DoCommand({"setup_station": true})`. This
   captures a point cloud, filters it, runs Poisson reconstruction +
   decimation, segments the resulting mesh into bins, and writes the
   assets `merged.pcd`, `filtered_merged.pcd`, `mesh.ply`, and
   `zones.json` into `/home/viam/assets/`.
4. Inspect `zones.json` (e.g. via `build-coordinator.DoCommand({"get_setup_result": true})`)
   and assign each ingredient a `zone-id` matching the right zone.
5. Restart or reconfigure so `grabber-controls` picks up the new bin
   layout, then call `build-coordinator.DoCommand({"build_salad": {...}})`.
