# Model ncs:salad:file-vision

A vision service that loads a single `.ply` mesh file at startup and
exposes it as a point cloud + mesh geometry through the vision API. Used
to back the front-end's "view the bin mesh" panel against
`/home/viam/assets/mesh.ply` without round-tripping a live camera.

Only `GetObjectPointClouds` is implemented; the returned `vision.Object`
has both `PointCloud` (mesh sampled at density `1`) and `Geometry` (the
raw mesh) set.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "file": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `file` | string | Required at runtime | Filesystem path to a `.ply` mesh file. The path is not validated by `Validate` but the constructor calls `NewMeshFromPLYFile`, so an empty or invalid path will fail at startup. |

### Example Configuration

```json
{
  "file": "/home/viam/assets/mesh.ply"
}
```

## API

This model implements `rdk:service:vision`, but only the
`GetObjectPointClouds` method:

- `GetObjectPointClouds(ctx, cameraName, extra)` — returns a single
  `vision.Object` whose `PointCloud` is sampled from the mesh at density
  `1` and whose `Geometry` is the mesh itself. The `cameraName` argument
  is ignored.

All other vision methods return `not implemented`.

## DoCommand

Not implemented; calls return `not implemented`.
