# Model ncs:salad:grabber-controls

A generic service that drives a single arm + gripper to grab ingredients
out of segmented bins and drop them into the salad bowl. It is the
ingredient-handling counterpart to `ncs:salad:bowl-controls`.

Grab poses are not configured by hand — they are computed at runtime from
the segmented mesh produced by `build-coordinator.setup_station`. On the
first command after startup, this model lazy-loads `mesh.ply` and
`zones.json` from `assets-dir` (default `/home/viam/assets`) and builds:

- A world obstacle (octree derived from `mesh.ply`) that is passed as
  `WorldState` to every motion plan.
- A hover pose per configured bin, centered on the bin's segmentation
  centroid at height `Z_mean + bin-hover-height-mm` with orientation
  `bin-hover-orientation`.
- A grab pose per configured bin, centered on the bin's centroid at
  `Z_min + grab-height-mm - depth-offset-mm` with orientation
  `bin-hover-orientation`. The bin is then re-entered with orientation
  `grab-orientation` (a "tilt") so the gripper can scoop sideways.

The bowl drop-off is driven by an inline `dropping-pose` (position + orientation)
configured directly on this service. A hover pose is computed at startup as
`dropping-pose` offset upward by `bowl-hover-height-mm` (default 150 mm). Both
moves go through the motion planner with the bin mesh as a world obstacle.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "arm": <string>,
  "gripper": <string>,
  "motion-service": <string>,
  "dropping-pose": { "x": <float>, "y": <float>, "z": <float>, "orientation": { "x": <float>, "y": <float>, "z": <float>, "th": <float> } },
  "bowl-hover-height-mm": <float>,
  "left-home": <string>,
  "bins": [
    { "name": <string>, "zone-id": <int> }
  ],
  "bin-hover-height-mm": <float>,
  "bin-hover-orientation": { "x": <float>, "y": <float>, "z": <float>, "th": <float> },
  "grab-height-mm": <float>,
  "grab-orientation": { "x": <float>, "y": <float>, "z": <float>, "th": <float> },
  "shake-arm-service": <string>,
  "assets-dir": <string>,
  "x-offset-mm": <float>,
  "y-offset-mm": <float>,
  "save-plans": <bool>,
  "grab-line-tolerance-mm": <float>,
  "grab-orientation-tolerance-degs": <float>,
  "enable-bin-clearance": <bool>,
  "bin-clearance-x-offset-mm": <float>,
  "bin-clearance-z-offset-mm": <float>,
  "clearance-line-tolerance-mm": <float>,
  "clearance-orientation-tolerance-degs": <float>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arm` | string | Required | Name of the `arm` component this service moves. |
| `gripper` | string | Required | Name of the `gripper` component on the arm. |
| `motion-service` | string | Required | Name of the `motion` service used to plan moves with the bin mesh as a world obstacle. |
| `dropping-pose` | object | Required | Inline pose (fields: `x`, `y`, `z` in mm, `orientation` as OV-degrees) where the gripper opens to drop the ingredient into the bowl. |
| `bowl-hover-height-mm` | float | Optional | Height (mm) added to `dropping-pose.z` to compute the hover pose used before and after dropping. Defaults to `150`. |
| `left-home` | string | Required | Name of a `switch` used by `reset` to send the arm home. |
| `bins` | array | Required | Map from bin name to segmentation zone ID. Must be non-empty; each entry must have a non-empty `name`. The `zone-id` must exist in `zones.json` at runtime. |
| `bin-hover-height-mm` | float | Required | Hover height above the bin's mean Z (mm). Used for both pre-grab approach and post-grab return. Must be non-zero. |
| `bin-hover-orientation` | orientation vector (degrees) | Required | Orientation used at the hover pose and for the straight descend/ascend through the bin. Must be specified. |
| `grab-height-mm` | float | Required (implicit) | Offset (mm) added to the bin's min Z when computing the grab depth. Negative values descend below the bin floor. |
| `grab-orientation` | orientation vector (degrees) | Required | Orientation used to "tilt" the gripper inside the bin before closing. Must be specified. |
| `shake-arm-service` | string | Optional | Name of a generic service that accepts `{"shake_arm": true}`. Called after dropping the ingredient into the bowl. |
| `assets-dir` | string | Optional | Directory containing `mesh.ply` and `zones.json`. Defaults to `/home/viam/assets`. |
| `x-offset-mm` | float | Optional | Global X offset (mm) applied to every computed hover and grab pose. Useful for fine-tuning without re-running setup. Defaults to `0`. |
| `y-offset-mm` | float | Optional | Global Y offset (mm) applied to every computed hover and grab pose. Defaults to `0`. |
| `save-plans` | bool | Optional | When `true`, every `get_from_bin` call writes a JSON plan record (waypoints, linearity, error) to `/root/.viam/capture/grab-<ts>-zone<id>.json`. Defaults to `false`. |
| `grab-line-tolerance-mm` | float | Optional | Linear-constraint tolerance (mm) on the straight descend and ascend through the bin. Defaults to `1.0`. |
| `grab-orientation-tolerance-degs` | float | Optional | Orientation-constraint tolerance (degrees) on the descend/ascend. Defaults to `1.0`. |
| `enable-bin-clearance` | bool | Optional | When `true`, after ascending back to hover the arm makes an extra linear move offset by `bin-clearance-x-offset-mm` and/or `bin-clearance-z-offset-mm` for fridge clearance before the bowl-delivery swing. Defaults to `false`. |
| `bin-clearance-x-offset-mm` | float | Optional | Lateral X shift (mm) from hover for the clearance move. At least one of `bin-clearance-x-offset-mm` or `bin-clearance-z-offset-mm` must be non-zero when `enable-bin-clearance` is `true`. |
| `bin-clearance-z-offset-mm` | float | Optional | Vertical Z shift (mm) from hover for the clearance move. A combined X+Z move gives the planner more freedom to satisfy orientation constraints. |
| `clearance-line-tolerance-mm` | float | Optional | Linear-constraint tolerance (mm) on the clearance move. Defaults to `1.0`. |
| `clearance-orientation-tolerance-degs` | float | Optional | Orientation-constraint tolerance (degrees) on the clearance move. Defaults to `45.0`. |

### Example Configuration

```json
{
  "arm": "left-arm",
  "gripper": "left-gripper",
  "motion-service": "builtin",
  "dropping-pose": {
    "x": 361.75, "y": 529.73, "z": 534.00,
    "orientation": { "x": 0.084, "y": 0.037, "z": -0.996, "th": 111.76 }
  },
  "bowl-hover-height-mm": 150,
  "left-home": "left-home",
  "shake-arm-service": "left-shake",
  "bins": [
    { "name": "lettuce",  "zone-id": 0 },
    { "name": "chicken",  "zone-id": 1 },
    { "name": "tomato",   "zone-id": 2 },
    { "name": "croutons", "zone-id": 3 }
  ],
  "bin-hover-height-mm": 100,
  "bin-hover-orientation": { "x": 0, "y": 0, "z": -1, "th": 0 },
  "grab-height-mm": -30,
  "grab-orientation": { "x": 0.5, "y": 0, "z": -1, "th": 0 },
  "enable-bin-clearance": true,
  "bin-clearance-x-offset-mm": -25,
  "bin-clearance-z-offset-mm": 20
}
```

## DoCommand

`DoCommand` accepts exactly one of the following top-level keys. Any other
key results in `unknown command, expected 'get_from_bin' or 'bin_hover' field`.

### get_from_bin

Plans and executes a full grab cycle for a single bin:

1. Move to hover pose above the bin.
2. Open gripper.
3. Linearly descend to the grab pose.
4. Tilt to `grab-orientation`.
5. Close gripper (`Grab`).
6. Un-tilt back to the grab pose orientation.
7. Linearly ascend back to hover pose.
8. (If `enable-bin-clearance`) move to the clearance pose.
9. Move arm to bowl hover pose (`dropping-pose` + `bowl-hover-height-mm`).
10. Move arm to `dropping-pose`.
11. Open gripper to release.
12. Move arm back to bowl hover pose.
13. (If `shake-arm-service` is configured) call `{"shake_arm": true}`.

The required `get_from_bin` value is the integer zone ID to grab from
(must be one of the `zone-id`s in `bins`). The optional `depth-offset-mm`
descends an additional N millimeters below the configured grab Z; it must
be non-negative. If motion planning fails ("`physically unreachable`",
"`zero IK solutions`", "`no plan found`", or "`fatal early collision`")
and `depth-offset-mm > 0`, the build coordinator halves it and retries.

Request:

```json
{
  "get_from_bin": 2,
  "depth-offset-mm": 0
}
```

Response:

```json
{
  "success": true,
  "bin": "tomato",
  "depth-offset-mm": 0,
  "message": "Successfully grabbed from bin 'tomato' and moved to bowl"
}
```

### bin_hover

Moves the arm to the hover pose for a given zone ID without descending or
grabbing. Useful for teaching/testing.

Request:

```json
{ "bin_hover": 2 }
```

The response is `null` on success; an error is returned if the zone ID is
unknown or its hover pose is not yet computed.

### reset

Sends the arm home by setting the `left-home` switch to position `2`.

Request:

```json
{ "reset": true }
```

The response is `null` on success.
