# Model ncs:salad:chefs-kiss-controls

A generic service that performs a short "chef's kiss" seasoning gesture
over the bowl. It is called once at the end of every successful
`build_salad` flow.

The sequence is: close gripper → drive `position` switch to `2` (over the
bowl) → 200 ms pause → open gripper to drop seasoning → call
`arm-shaker-service` with `{"shake_arm": true}` → drive `home` switch to
`2` to send the arm home.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "position": <string>,
  "gripper": <string>,
  "home": <string>,
  "arm-shaker-service": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `position` | string | Required | Name of a `switch` representing the pose over the bowl where the gesture happens. |
| `gripper` | string | Required | Name of a `gripper` that holds the seasoning ingredient between gestures and opens to release it over the bowl. |
| `home` | string | Required | Name of a `switch` used to send the arm home after the gesture. |
| `arm-shaker-service` | string | Required | Name of a generic service that accepts `{"shake_arm": true}`. Called to shake the arm while the gripper is open over the bowl. |

### Example Configuration

```json
{
  "position": "over-bowl",
  "gripper": "right-gripper",
  "home": "right-home",
  "arm-shaker-service": "right-shake"
}
```

## DoCommand

`DoCommand` accepts exactly one top-level key. Any other key results in
`unknown command, expected 'chefs_kiss' field`.

### chefs_kiss

Runs the chef's kiss gesture described above.

Request:

```json
{ "chefs_kiss": true }
```

Response:

```json
{
  "success": true,
  "message": "Successfully completed chefs kiss"
}
```
