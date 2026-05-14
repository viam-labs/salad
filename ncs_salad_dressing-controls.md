# Model ncs:salad:dressing-controls

A generic service that drives a single gripper through a fixed pour
sequence to dispense salad dressing over the delivered bowl. Multiple
dressing bottles are supported — the prepare / pour / post-pour / home
poses are shared across every dressing, while each entry in `dressings`
has its own `approach-grab` and `grab` poses so different bottles can be
picked up from and returned to different slots.

The pour sequence drives the gripper to: open → approach the bottle →
descend onto it → close → lift back to approach → `prepare-dressing` →
`pour-dressing` (optional shake) → `pour-dressing2` (optional shake) →
back to `pour-dressing` (optional shake) → `post-pour-dressing` →
`prepare-dressing` → approach the bottle slot → descend onto it → open
gripper to release → lift back to approach → `home`.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "gripper": <string>,
  "prepare-dressing": <string>,
  "pour-dressing": <string>,
  "pour-dressing2": <string>,
  "post-pour-dressing": <string>,
  "home": <string>,
  "shake-arm-service": <string>,
  "dressings": {
    "<name>": {
      "approach-grab": <string>,
      "grab": <string>
    }
  }
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `gripper` | string | Required | Name of the `gripper` used to pick up and put down dressing bottles. |
| `prepare-dressing` | string | Required | Name of a `switch` representing the "ready to pour" pose above the bowl. |
| `pour-dressing` | string | Required | Name of a `switch` representing the first pour pose. |
| `pour-dressing2` | string | Required | Name of a `switch` representing the second pour pose (different tilt or position). |
| `post-pour-dressing` | string | Required | Name of a `switch` representing the "shake-off" pose after pouring. |
| `home` | string | Required | Name of a `switch` used at the end of every pour and by `reset`. |
| `shake-arm-service` | string | Optional | Name of a generic service that accepts `{"shake_arm": true}`. When set, the service is called three times during a pour (at each pour pose) to agitate the bottle. |
| `dressings` | object | Required | Map of dressing options keyed by ingredient name. Must be non-empty. Each entry has `approach-grab` and `grab` switch names; both are required. The name should match an ingredient with category `"dressing"` in the `build-coordinator` config. |

### Example Configuration

```json
{
  "gripper": "right-gripper",
  "prepare-dressing": "prepare-dressing",
  "pour-dressing": "pour-dressing",
  "pour-dressing2": "pour-dressing2",
  "post-pour-dressing": "post-pour-dressing",
  "home": "right-home",
  "shake-arm-service": "right-shake",
  "dressings": {
    "ranch": {
      "approach-grab": "approach-ranch",
      "grab": "grab-ranch"
    },
    "italian": {
      "approach-grab": "approach-italian",
      "grab": "grab-italian"
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

Sends the arm to the `home` switch (position `2`).

Request:

```json
{ "reset": true }
```

The response is `null` on success.
