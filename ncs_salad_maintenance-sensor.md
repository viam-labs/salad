# Model ncs:salad:maintenance-sensor

A sensor component that exposes whether it is currently safe for a human
to enter the salad station. The sensor wraps an `ncs:salad:build-coordinator`
generic service: every `Readings` call issues `DoCommand({"status": true})`
against the coordinator and inspects the returned `status` string.

A build is considered "busy" — and the station unsafe — whenever the
coordinator's `status` is anything other than `""`, `"idle"`, or
`"complete"`. This is the canonical signal for "salad-build in progress"
and is intended to be wired into Viam's [maintenance window](https://docs.viam.com/manage/reference/viam-server/#maintenance-window-configuration)
configuration so that automatic reconfigurations or restarts do not
interrupt an in-flight salad.

## Requirements

- A configured `ncs:salad:build-coordinator` generic service whose name
  matches `build-coordinator-name`.

## Configuration

The following attribute template can be used to configure this model:

```json
{
  "build-coordinator-name": <string>
}
```

### Attributes

The following attributes are available for this model:

| Name | Type | Inclusion | Description |
|---|---|---|---|
| `build-coordinator-name` | string | Required | Name of the `ncs:salad:build-coordinator` generic service to query for status. Declared as a required dependency so the sensor cannot start without it. |

### Example Configuration

```json
{
  "build-coordinator-name": "build-coordinator"
}
```

## API

This model implements the standard
[`rdk:component:sensor`](https://docs.viam.com/components/sensor/) API.

`Readings` returns:

| Key | Type | Description |
|---|---|---|
| `is_safe` | bool | `true` when the build coordinator's `status` is empty, `"idle"`, or `"complete"`. `false` otherwise. |
| `build_status` | string | The raw `status` string from the build coordinator (`"idle"`, `"preparing"`, `"adding lettuce"`, `"delivering salad"`, `"complete"`, `"stopped"`, `"failed"`, etc.). |

If the coordinator returns an error, `Readings` returns that error.

## DoCommand

This model does not implement `DoCommand`; calling it returns an empty
result and no error.
