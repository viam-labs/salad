# Model ncs:salad:dressing-controls

A generic service that pre-plans all arm trajectories upfront using motion
planning with the bin mesh as a collision obstacle, then executes them in
sequence via `MoveThroughJointPositions`. Multiple dressing bottles are
supported — the prepare / pour / post-pour / home poses are shared across
every dressing, while each entry in `dressings` has its own `approach-grab`
and `grab` poses for different bottle slots.

The pour sequence: open gripper → approach bottle → descend onto bottle
(slow) → close gripper → lift → `prepare-dressing` → `pour-dressing`
(optional shake) → `post-pour-dressing` → `prepare-dressing` → approach
bottle slot → descend onto slot (slow) → open gripper → lift → `home`.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "arm": <string>,
  "gripper": <string>,
  "assets-dir": <string>,
  "grab-speed-degs-per-sec": <float>,
  "prepare-dressing":   { "point": {}, "orientation": {}, "constraints": {} },
  "pour-dressing":      { "point": {}, "orientation": {}, "constraints": {} },
  "post-pour-dressing": { "point": {}, "orientation": {}, "constraints": {} },
  "home":               { "point": {}, "orientation": {}, "constraints": {} },
  "shake-arm-service": <string>,
  "dressings": {
    "<name>": {
      "approach-grab": { "point": {}, "orientation": {}, "constraints": {} },
      "grab":          { "point": {}, "orientation": {}, "constraints": {} }
    }
  }
}
```

### Attributes

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `arm` | string | Required | Name of the `arm` component. |
| `gripper` | string | Required | Name of the `gripper` used to pick up and put down dressing bottles. |
| `assets-dir` | string | Optional | Directory containing `mesh.ply` used as a collision obstacle during planning. Defaults to `/home/viam/assets`. |
| `grab-speed-degs-per-sec` | float | Optional | Max joint velocity in deg/s for the two bottle-contact steps (`grab` and `grab_return`). Defaults to `30`. All other steps run at the arm's default speed. |
| `prepare-dressing` | pose | Required | Pose above the bowl, used before and after pouring. |
| `pour-dressing` | pose | Required | Tilt pose used to pour dressing into the bowl. |
| `post-pour-dressing` | pose | Required | Pose moved to after pouring (e.g. to shake off excess). |
| `home` | pose | Required | Resting pose used at the end of every pour and by `reset`. |
| `shake-arm-service` | string | Optional | Name of a generic service that accepts `{"shake_arm": true}`. Called once after the pour step. |
| `dressings` | object | Required | Map of dressing options keyed by ingredient name. Must be non-empty. Each entry requires `approach-grab` and `grab` poses. The name must match an ingredient with category `"dressing"` in the `build-coordinator` config. |

Each pose object has the following fields:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `point` | object | Required | `{"X": <mm>, "Y": <mm>, "Z": <mm>}` |
| `orientation` | object | Required | Orientation vector degrees: `{"x", "y", "z", "th"}` |
| `constraints` | object | Optional | `motionplan.Constraints` applied when planning the move to this pose. |

### Example Configuration

```json
{
  "arm": "right-arm",
  "gripper": "right-gripper",
  "grab-speed-degs-per-sec": 30,
  "prepare-dressing": {
    "point": {"X": 238.2, "Y": 609.6, "Z": 357.2},
    "orientation": {"x": 0.875, "y": 0.322, "z": -0.361, "th": -156.1},
    "constraints": {"linear_constraints": [{"LineToleranceMm": 0, "OrientationToleranceDegs": 0}]}
  },
  "pour-dressing": {
    "point": {"X": 241.1, "Y": 607.6, "Z": 358.0},
    "orientation": {"x": 0.881, "y": 0.285, "z": -0.377, "th": -62.3},
    "constraints": {"linear_constraints": [{"LineToleranceMm": 10, "OrientationToleranceDegs": 0}]}
  },
  "post-pour-dressing": {
    "point": {"X": 238.2, "Y": 609.6, "Z": 357.2},
    "orientation": {"x": 0.875, "y": 0.322, "z": -0.361, "th": -156.1},
    "constraints": {"linear_constraints": [{"LineToleranceMm": 0, "OrientationToleranceDegs": 0}]}
  },
  "home": {
    "point": {"X": 391.2, "Y": -92.8, "Z": 600},
    "orientation": {"x": 0.232, "y": 0.043, "z": -0.972, "th": -170.8}
  },
  "shake-arm-service": "right-dressing-shake",
  "dressings": {
    "vinaigrette": {
      "approach-grab": {
        "point": {"X": 300, "Y": 75, "Z": 500},
        "orientation": {"x": 0.967, "y": 0.053, "z": -0.251, "th": 179.2},
        "constraints": {"linear_constraints": [{}]}
      },
      "grab": {
        "point": {"X": 488.6, "Y": 95.7, "Z": 420},
        "orientation": {"x": 0.977, "y": 0.016, "z": -0.214, "th": 180.0},
        "constraints": {"linear_constraints": [{"LineToleranceMm": 0, "OrientationToleranceDegs": 5}]}
      }
    }
  }
}
```

## DoCommand

`DoCommand` accepts exactly one of the top-level keys below. Any other
key results in `unknown command, expected 'pour_dressing' or 'reset' field`.

### pour_dressing

Runs the full pour sequence for a single dressing. The value must be the
name of an entry in the `dressings` map; an unknown name results in an
error.

Request:

```json
{ "pour_dressing": "ranch" }
```

Response:

```json
{
  "success": true,
  "message": "Successfully poured dressing"
}
```

### reset

Plans and executes a single move to the `home` pose.

Request:

```json
{ "reset": true }
```

The response is `null` on success.
