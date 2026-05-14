# Model ncs:salad:supply-detector

A generic service that estimates whether each ingredient bin is running
low by inspecting a rectangular region of interest (ROI) on a single
overhead camera frame. For each configured ROI the service counts what
fraction of pixels do **not** look like the bin's bare steel surface and
compares the fraction to `low_threshold`.

A pixel is considered "steel" when it is simultaneously bright
(`(R + G + B) / 3 > 150`) and low-saturation (`max(R,G,B) - min(R,G,B) < 30`),
i.e. close to a neutral gray/silver. The complement of that ratio is the
"non-steel" ratio. When the non-steel ratio falls below `low_threshold` the
bin is reported as `"low"`; otherwise it is reported as `"available"`. If
the ROI is invalid (e.g. fully outside the image or empty) the bin is
reported as `"unknown"`.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "camera": <string>,
  "bins": [
    {
      "name": <string>,
      "x1": <int>,
      "y1": <int>,
      "x2": <int>,
      "y2": <int>
    }
  ],
  "low_threshold": <float>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `camera` | string | Required | Name of the `camera` component to capture frames from. Frames are decoded as `image/jpeg`. |
| `bins` | array | Required | Non-empty list of ROIs to monitor. Each entry has `name`, `x1`, `y1`, `x2`, `y2`. `name` must be non-empty; `x2 > x1` and `y2 > y1`. ROIs are clamped to the image bounds at evaluation time. |
| `low_threshold` | float | Optional | Non-steel-pixel fraction below which a bin is considered `"low"`. Defaults to `0.20`. Note that if you explicitly set this to `0`, the service treats it as unset and resets it to `0.20`. |

### Example Configuration

```json
{
  "camera": "overhead-cam",
  "low_threshold": 0.20,
  "bins": [
    { "name": "lettuce",  "x1": 100, "y1": 100, "x2": 300, "y2": 300 },
    { "name": "chicken",  "x1": 320, "y1": 100, "x2": 520, "y2": 300 },
    { "name": "tomato",   "x1": 540, "y1": 100, "x2": 740, "y2": 300 },
    { "name": "croutons", "x1": 760, "y1": 100, "x2": 960, "y2": 300 }
  ]
}
```

## DoCommand

`DoCommand` accepts exactly one top-level key. Any other key results in
`unknown command, expected 'check_supply'`.

### check_supply

Captures one frame from the configured camera and evaluates every ROI.

Request:

```json
{ "check_supply": true }
```

Response — a map from bin name to its current state:

```json
{
  "lettuce":  "available",
  "chicken":  "low",
  "tomato":   "available",
  "croutons": "unknown"
}
```

Possible per-bin values:

- `"available"` — non-steel ratio is at or above `low_threshold`.
- `"low"` — non-steel ratio is below `low_threshold`.
- `"unknown"` — the ROI could not be evaluated (e.g. fully outside the
  image after clamping).
