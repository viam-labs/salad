# Model ncs:salad:build-coordinator

A generic service that orchestrates automated salad assembly. The build
coordinator owns the end-to-end build flow:

1. Reads the configured `scale-sensor` before and after each grab to verify
   that ingredient weight was added to the bowl.
2. Drives a `grabber-controls` generic service to pick ingredients out of
   bins and drop them into the bowl, sorted by category in the order
   `base` → `protein` → `topping` → `dressing`.
3. Drives a `bowl-controls` generic service to grab a fresh bowl, deliver
   the finished bowl, and reset the right arm.
4. Drives a `dressing-controls` generic service to pour each configured
   dressing after the bowl has been delivered.
5. Calls a `chefs-kiss-controls` generic service at the end of every build.
6. Provides a separate `setup_station` flow that captures a point cloud
   from an overhead camera and writes `merged.pcd`, `filtered_merged.pcd`,
   `mesh.ply`, and `zones.json` into `/home/viam/assets/`. The grabber uses
   these assets to compute hover and grab poses for each ingredient bin.

When a grab adds no weight (less than `0.5g` of change on the scale), the
coordinator probes deeper into the bin on the next attempt by
`depth-step-mm`, capped at `max-depth-offset-mm`. If motion planning rejects
a depth as unreachable, the offset is halved and retried (binary search for
the deepest reachable Z). After 3 consecutive grabs with no weight change,
the ingredient errors out (likely an empty bin).

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "grabber-controls": <string>,
  "bowl-controls": <string>,
  "dressing-controls": <string>,
  "chefs-kiss-controls": <string>,
  "scale-sensor": <string>,
  "ingredients": [
    {
      "name": <string>,
      "grams-per-serving": <float>,
      "category": "base" | "protein" | "topping" | "dressing",
      "zone-id": <int>
    }
  ],
  "text-to-speech": <string>,
  "imaging-camera": <string>,
  "capture-dir": <string>,
  "simulate": <bool>,
  "skip-lil-arm": <bool>,
  "depth-step-mm": <float>,
  "max-depth-offset-mm": <float>,
  "mesh-target-triangles": <int>,
  "filter": {
    "voxel-mm": <float>,
    "neighbor-radius": <int>,
    "min-neighbors": <int>,
    "min-component-voxels": <int>
  },
  "segmentation": {
    "cell-size-mm": <float>,
    "divider-z-percentile": <float>,
    "divider-gradient-mm": <float>,
    "divider-dilation": <int>,
    "min-zone-area-mm2": <float>,
    "max-zone-area-mm2": <float>
  }
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `grabber-controls` | string | Required | Name of an `ncs:salad:grabber-controls` generic service. |
| `bowl-controls` | string | Required | Name of an `ncs:salad:bowl-controls` generic service. |
| `dressing-controls` | string | Required | Name of an `ncs:salad:dressing-controls` generic service. |
| `chefs-kiss-controls` | string | Required | Name of an `ncs:salad:chefs-kiss-controls` generic service. Called once at the end of every successful build. |
| `scale-sensor` | string | Required | Name of a `sensor` component whose `Readings` returns at least one numeric value (interpreted as the current scale weight in grams). |
| `ingredients` | array | Required | Catalog of ingredients available to be built. Must be non-empty. See below. |
| `text-to-speech` | string | Optional | Name of a `viam:beanjamin:text-to-speech` service (or anything that accepts `{"say": "<text>"}` via `DoCommand`). When set, the coordinator announces completion after each successful build. |
| `imaging-camera` | string | Optional | Name of a `camera` component used by `setup_station` to capture a point cloud. Required to call `setup_station`. |
| `capture-dir` | string | Optional | Filesystem directory for timestamped capture outputs (PCDs, meshes, zones). Defaults to `/root/.viam/capture`. |
| `simulate` | bool | Optional | When `true`, `build_salad` skips all robot commands and immediately reports success. Defaults to `false`. |
| `skip-lil-arm` | bool | Optional | When `true`, the coordinator skips the optional lil-arm `grab_bowl`/`grab_lid` steps and tells `bowl-controls.reset` not to send the lil-arm home. Useful when the lil-arm is unreliable but you still want it configured in `bowl-controls`. Defaults to `false`. |
| `depth-step-mm` | float | Optional | Millimeters to descend deeper into the bin on each empty-handed grab retry. Set to `0` or negative to disable depth probing. Defaults to `20`. |
| `max-depth-offset-mm` | float | Optional | Cap on how many millimeters below the configured grab Z the depth probe is allowed to reach. Set to `0` or negative to disable depth probing. Defaults to `80`. |
| `mesh-target-triangles` | int | Optional | Decimate the Poisson-reconstructed mesh down to roughly this many triangles using quadric error metrics. The mesh is loaded as a world obstacle for every motion plan, so a smaller mesh means faster planning. Set to `0` to disable decimation. Must be `>= 0`. Defaults to `5000`. |
| `filter` | object | Optional | Voxel filter parameters applied to the merged PCD before meshification (see below). Defaults are tuned against typical fridge captures. |
| `segmentation` | object | Optional | Bin segmentation tuning parameters applied to the reconstructed mesh (see below). |

Each entry in `ingredients` has the following fields:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `name` | string | Required | Ingredient name. Referenced from `build_salad` and from the `dressings` map in `dressing-controls`. |
| `grams-per-serving` | float | Required | Must be positive. Target weight added per serving. |
| `category` | string | Required | One of `"base"`, `"protein"`, `"topping"`, `"dressing"`. Drives build order. |
| `zone-id` | int | Required | Numeric zone ID from `zones.json` (produced by `setup_station`). Identifies which segmented bin to grab from. |

`filter` fields (used by `setup_station`):

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `voxel-mm` | float | Optional | Voxel size in millimeters. Must be `> 0`. |
| `neighbor-radius` | int | Optional | Half-side of the `(2r+1)^3` neighborhood used by the neighbor-count pass. Must be `>= 1`. |
| `min-neighbors` | int | Optional | Drop a voxel if its neighborhood has fewer than this many occupied voxels. Set to `0` to disable the neighbor-count pass. Must be `>= 0`. |
| `min-component-voxels` | int | Optional | After the neighbor-count pass, drop any 26-connected component smaller than this. Set to `0` to disable the connected-components pass. Must be `>= 0`. |

`segmentation` fields (used by `setup_station`):

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `cell-size-mm` | float | Optional | Grid cell size in millimeters used by the bin segmenter. Must be `> 0`. |
| `divider-z-percentile` | float | Optional | Percentile of cell Z heights used to detect bin dividers. Must be in `(0, 1]`. |
| `divider-gradient-mm` | float | Optional | Gradient threshold (mm) for divider detection. Must be `>= 0`. |
| `divider-dilation` | int | Optional | Number of dilation passes applied to the detected divider mask. Must be `>= 0`. |
| `min-zone-area-mm2` | float | Optional | Drop segmented zones smaller than this area in mm². Must be `> 0`. |
| `max-zone-area-mm2` | float | Optional | Drop segmented zones larger than this area in mm². Must be `>= 0`. |

### Example Configuration

```json
{
  "grabber-controls": "grabber-controls",
  "bowl-controls": "bowl-controls",
  "dressing-controls": "dressing-controls",
  "chefs-kiss-controls": "chefs-kiss-controls",
  "scale-sensor": "scale",
  "imaging-camera": "overhead-cam",
  "text-to-speech": "tts",
  "depth-step-mm": 20,
  "max-depth-offset-mm": 80,
  "mesh-target-triangles": 5000,
  "filter": {
    "voxel-mm": 10,
    "neighbor-radius": 1,
    "min-neighbors": 8,
    "min-component-voxels": 1000
  },
  "ingredients": [
    { "name": "lettuce",  "grams-per-serving": 30.0, "category": "base",    "zone-id": 0 },
    { "name": "chicken",  "grams-per-serving": 40.0, "category": "protein", "zone-id": 1 },
    { "name": "tomato",   "grams-per-serving": 25.0, "category": "topping", "zone-id": 2 },
    { "name": "croutons", "grams-per-serving": 15.0, "category": "topping", "zone-id": 3 },
    { "name": "ranch",    "grams-per-serving": 20.0, "category": "dressing","zone-id": -1 }
  ]
}
```

## DoCommand

`DoCommand` accepts exactly one of the following top-level keys. Any other
key results in `unknown command, expected 'build_salad', 'setup_station',
'stop', 'reset', 'status', 'list_ingredients', or 'get_setup_result' field`.

### build_salad

Builds and delivers a salad. The value is a map of ingredient name to
number of servings. Ingredients are sorted by category (base → protein →
topping → dressing) before being added. Each non-dressing ingredient is
grabbed repeatedly until the scale shows that `servings * grams-per-serving`
has been added. After all non-dressing ingredients are added the bowl is
delivered, then each configured dressing is poured at the delivery
position. Finally `chefs_kiss` is called.

An optional `customer_name` string is shown in the UI during the build and,
if `text-to-speech` is configured, included in the announcement
("`<name>`'s salad is ready!").

Only one operation may be in flight at a time; if `build_salad` is called
while another build or `setup_station` is in progress, it returns
`{"success": false, "message": "An operation is already in progress, ..."}`.

Request:

```json
{
  "build_salad": {
    "lettuce": 2,
    "tomato": 1,
    "croutons": 1
  },
  "customer_name": "Alice"
}
```

Response:

```json
{
  "success": true,
  "message": "Salad built and delivered successfully"
}
```

### setup_station

Captures a point cloud from `imaging-camera`, applies the configured
filter, runs Poisson reconstruction + decimation via the meshifier, and
segments the resulting mesh into bins. Writes the following stable files
into `/home/viam/assets/`:

- `merged.pcd` — raw merged point cloud captured from the camera.
- `filtered_merged.pcd` — point cloud after the filter passes; this is what
  gets meshed.
- `mesh.ply` — surface mesh (Poisson reconstruction, optionally decimated
  to `mesh-target-triangles`).
- `zones.json` — per-bin segmentation of the mesh.

Timestamped copies of each artifact are also written under `capture-dir`.
Requires `imaging-camera` to be configured.

Request:

```json
{ "setup_station": true }
```

Response on success:

```json
{
  "success": true,
  "message": "Station setup complete"
}
```

### get_setup_result

Returns the most recent setup artifacts. Errors if `setup_station` has
never run, if any of `merged.pcd`, `zones.json`, or `mesh.ply` is missing,
or if any configured ingredient has a `zone-id` that does not appear in
`zones.json`.

Request:

```json
{ "get_setup_result": true }
```

Response:

```json
{
  "pcd": "<base64-encoded merged.pcd bytes>",
  "zones": { ...zones.json contents... }
}
```

### stop

Cancels the currently running `build_salad` or `setup_station`. Returns
immediately if no operation is in progress. After cancelling a build, the
coordinator runs `resetAll` (sends grabber + bowl arms home) before
returning.

Request:

```json
{ "stop": true }
```

Response:

```json
{
  "success": true,
  "message": "Operation stopped"
}
```

### reset

Runs the standard reset sequence: `grabber-controls.reset` followed by
`bowl-controls.reset` (passing `skip-lil-arm` through).

Request:

```json
{ "reset": true }
```

Response:

```json
{
  "success": true,
  "message": "Successfully reset all controls"
}
```

### status

Returns the current build state. `progress` is the percentage of total
build steps completed (`total servings + 1 for bowl delivery`). `status`
is one of `"idle"`, `"preparing"`, `"setting_up_station"`,
`"adding <ingredient>"`, `"delivering salad"`, `"complete"`, `"stopped"`,
or `"failed"`. `customer_name` is empty unless a build is currently
running with a customer name.

Request:

```json
{ "status": true }
```

Response:

```json
{
  "status": "adding lettuce",
  "progress": 40.0,
  "customer_name": "Alice",
  "error_msg": ""
}
```

### list_ingredients

Returns the configured ingredient catalog.

Request:

```json
{ "list_ingredients": true }
```

Response:

```json
{
  "ingredients": [
    { "name": "lettuce",  "grams_per_serving": 30.0, "category": "base" },
    { "name": "chicken",  "grams_per_serving": 40.0, "category": "protein" },
    { "name": "tomato",   "grams_per_serving": 25.0, "category": "topping" },
    { "name": "croutons", "grams_per_serving": 15.0, "category": "topping" },
    { "name": "ranch",    "grams_per_serving": 20.0, "category": "dressing" }
  ]
}
```

## Tuning the setup filter

You can tune the filter against any captured PCD via the CLI before
changing config:

```bash
./bin/salad-cli filter --input <path>.pcd --viz \
  --voxel 10 --neighbor-radius 1 --min-neighbors 8 --min-component-voxels 1000
```
