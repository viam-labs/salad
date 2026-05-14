# Model ncs:salad:bowl-controls

A generic service that drives the "right" arm + gripper to fetch, deliver,
and reset the salad bowl, plus an optional secondary "lil-arm" used to
grab a bowl and a lid from stacks via switch-driven saved positions and a
force-controlled descent.

The right arm + gripper is driven entirely through saved-position
`switch` components (each is set to position `2` to drive to that saved
pose). The lil-arm, when configured, additionally uses an external
"xarm-force-mover" generic service for the force-driven descents during
`grab_bowl`, `grab_lid`, and `use_tool`.

`bowl-controls` also exposes a contact-detection routine
(`move_down_to_<name>`) that nudges the configured `little-arm` straight
down 2 mm at a time and uses the second value of `DoCommand({"load": true})`
on that arm to detect a sign flip (contact).

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "right-gripper": <string>,
  "right-above-bowl": <string>,
  "right-grab-bowl": <string>,
  "right-above-delivery": <string>,
  "right-bowl-delivery": <string>,
  "right-home": <string>,
  "little-arm": <string>,
  "lil-arm-gripper": <string>,
  "lil-arm-home": <string>,
  "lil-arm-poses": [
    {
      "name": <string>,
      "above": <string>,
      "at": <string>,
      "center-above": <string>,
      "center-at": <string>
    }
  ],
  "xarm-force-mover": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `right-gripper` | string | Required | Name of the `gripper` used to grab and release bowls. |
| `right-above-bowl` | string | Required | Name of a `switch` representing "above the bowl pickup point". |
| `right-grab-bowl` | string | Required | Name of a `switch` representing "down on the bowl pickup point". |
| `right-above-delivery` | string | Required | Name of a `switch` representing "above the bowl delivery point". |
| `right-bowl-delivery` | string | Required | Name of a `switch` representing "down on the bowl delivery point". |
| `right-home` | string | Required | Name of a `switch` used by `reset` to send the right arm home. |
| `little-arm` | string | Required | Name of an `arm` component used by the contact-detection routine `move_down_to_<name>`. Reads `load` from `DoCommand({"load": true})` to detect contact. |
| `lil-arm-gripper` | string | Optional | Name of a `gripper` for an auxiliary "lil-arm" used to grab a bowl and lid from a stack. If set, `lil-arm-home` and `lil-arm-poses` are also expected. |
| `lil-arm-home` | string | Required if `lil-arm-gripper` is set | Name of a `switch` used to send the lil-arm home. |
| `lil-arm-poses` | array | Optional | List of named pose groups for the lil-arm. Each entry has `name`, `above`, `at`, `center-above`, and `center-at` switches. The `grab_lid` command requires a pose named `"lid"`, `grab_bowl` requires `"bowl"`, and `use_tool` requires `"tool"`. All four switch fields are required on each entry. |
| `xarm-force-mover` | string | Optional | Name of a generic service implementing a force-driven move. Used as the descent step inside `grab_bowl`, `grab_lid`, `use_tool`, and `grab_and_use_tool`, and is the implementation of the `force_move` command. |

### Example Configuration

```json
{
  "right-gripper": "right-gripper",
  "right-above-bowl": "right-above-bowl",
  "right-grab-bowl": "right-grab-bowl",
  "right-above-delivery": "right-above-delivery",
  "right-bowl-delivery": "right-bowl-delivery",
  "right-home": "right-home",
  "little-arm": "lil-arm",
  "lil-arm-gripper": "lil-arm-gripper",
  "lil-arm-home": "lil-arm-home",
  "xarm-force-mover": "force-mover",
  "lil-arm-poses": [
    {
      "name": "lid",
      "above": "above-lid",
      "at": "at-lid",
      "center-above": "above-center",
      "center-at": "at-center"
    },
    {
      "name": "bowl",
      "above": "above-bowl-stack",
      "at": "at-bowl-stack",
      "center-above": "above-center",
      "center-at": "at-center"
    }
  ]
}
```

## DoCommand

`DoCommand` accepts exactly one of the top-level keys below. Any other
key results in `unknown command, expected 'deliver_bowl', 'prepare_bowl',
'grab_lid', 'grab_bowl', 'move_down_to_bowl', 'move_down_to_lid',
'force_move', 'use_tool', 'grab_and_use_tool', or 'reset' field`.

### deliver_bowl

Picks up the bowl from the pickup point under the ingredient area, swings
it over to the delivery point, and opens the gripper to release. Steps:
open gripper → `right-above-bowl` → `right-grab-bowl` → close gripper →
`right-above-bowl` → `right-above-delivery` → `right-bowl-delivery` →
gripper set position `400.0`.

Request:

```json
{ "deliver_bowl": true }
```

Response:

```json
{
  "success": true,
  "message": "Successfully delivered bowl"
}
```

### prepare_bowl

Fetches a bowl from the delivery position and places it under the
ingredient area. Steps: open gripper → `right-above-delivery` →
`right-bowl-delivery` → close gripper → `right-above-delivery` →
`right-above-bowl` → contact-detect descent to bowl → open gripper →
`right-above-bowl`.

Request:

```json
{ "prepare_bowl": true }
```

Response:

```json
{
  "success": true,
  "message": "Successfully prepared bowl"
}
```

### grab_bowl

Uses the lil-arm to grab a bowl from its stack and move it to the center
drop point. Requires `lil-arm-gripper` to be configured, a `lil-arm-poses`
entry named `"bowl"`, and an `xarm-force-mover` for the descent. The
`target` field is required and is forwarded to the force-mover as the
final Z target.

Request:

```json
{
  "grab_bowl": true,
  "target": 60
}
```

Response:

```json
{
  "success": true,
  "message": "Successfully grabbed bowl"
}
```

### grab_lid

Same as `grab_bowl` but uses the `"lid"` pose. Also requires `target`.

Request:

```json
{
  "grab_lid": true,
  "target": 80
}
```

Response:

```json
{
  "success": true,
  "message": "Successfully grabbed lid"
}
```

### use_tool

Uses the lil-arm to pick up the configured `"tool"` pose, carry it to the
center pose, apply a force_move (cmd is forwarded as-is to `xarm-force-mover`),
press it down on the center, then return the tool to its slot and the
lil-arm home. Requires a `lil-arm-poses` entry named `"tool"`, an
`xarm-force-mover`, and the fields `joint`, `axis`, and `target`.

Request:

```json
{
  "use_tool": true,
  "joint": 1,
  "axis": "z",
  "target": 50
}
```

Response:

```json
{
  "success": true,
  "message": "Successfully used tool"
}
```

### grab_and_use_tool

Convenience wrapper that runs `grab_bowl`, `grab_lid`, and `use_tool`
back-to-back, sharing a single `joint` and `axis` across all three
force-driven descents. `targets` is a 3-element array of numbers in the
order `[bowl, lid, tool]`. Requires `lil-arm-poses` entries named
`"bowl"`, `"lid"`, and `"tool"`.

Request:

```json
{
  "grab_and_use_tool": true,
  "joint": 1,
  "axis": "z",
  "targets": [60, 80, 50]
}
```

Response:

```json
{
  "success": true,
  "message": "Successfully grabbed bowl, grabbed lid, and used tool"
}
```

### move_down_to_bowl / move_down_to_lid

Drives the `little-arm` straight down 2 mm at a time and stops when the
second value of `DoCommand({"load": true})` on that arm flips sign
(interpreted as contact). The named `lil-arm-poses` entry's `at` switch is
set to position `2` before the loop starts. Requires the corresponding
`"bowl"` or `"lid"` pose to be configured.

Request:

```json
{ "move_down_to_bowl": true }
```

Response:

```json
{ "success": true }
```

### force_move

Direct passthrough to the configured `xarm-force-mover`. Forwards `joint`,
`axis`, and `target` fields verbatim. All three fields are required.

Request:

```json
{
  "force_move": true,
  "joint": 1,
  "axis": "z",
  "target": 50
}
```

Response: whatever the force-mover service returns.

### lil_arm_home

Sends the lil-arm to its home position. Requires `lil-arm-home` to be
configured.

Request:

```json
{ "lil_arm_home": true }
```

Response:

```json
{
  "success": true,
  "message": "Sent lil-arm home"
}
```

### reset

Sends the right arm home by setting `right-home` to position `2`. If
`lil-arm-home` is configured and `skip_lil_arm` is not `true`, also sends
the lil-arm home.

Request:

```json
{
  "reset": true,
  "skip_lil_arm": false
}
```

The response is `null` on success.
