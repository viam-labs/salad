# Model ncs:salad:passthrough-to-camera

A vision service that forwards point clouds from a single camera through
the `vision` API surface, so callers that only have a `vision` service
binding can still pull a point cloud out of a camera. Used by the front-end
to render the captured PCD during station setup.

Only `GetObjectPointClouds` is implemented — every other `vision.Service`
method (`Detections`, `Classifications`, `GetProperties`,
`CaptureAllFromCamera`, `DoCommand`) returns `not implemented`.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "camera": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `camera` | string | Required | Name of the `camera` component to read point clouds from. Declared as a required dependency. |

### Example Configuration

```json
{
  "camera": "overhead-cam"
}
```

## API

This model implements `rdk:service:vision`, but only the
`GetObjectPointClouds` method:

- `GetObjectPointClouds(ctx, cameraName, extra)` — calls `NextPointCloud`
  on the configured camera and returns a single
  `vision.Object` wrapping the result. The `cameraName` argument is
  ignored — the camera configured on this service is always used.

All other vision methods return `not implemented`.

## DoCommand

Not implemented; calls return `not implemented`.
