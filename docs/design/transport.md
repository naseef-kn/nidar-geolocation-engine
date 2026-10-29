# Transport design notes

## Message polling

`Drone.poll()` returns all pending messages as a list, draining the inbox in one call.

**Why a list:** one call per control-loop cycle, simple to read and to test.

**Known limit:** if messages arrive faster than a drone processes them, a single
poll can return a large list. This is not a concern at the current scale (two
drones and one GCS).

**Planned change:** add an optional `max_messages` argument to `poll()`. The default
will be "no limit", so existing callers keep working. Add it when message volume
or measured loop latency makes it necessary, not before.

## Endpoints

- `gcs` is reserved and registered automatically.
- Drone IDs follow the same format as in `drone_config.py`.
- Sending to or from an unregistered endpoint raises an error; it is never dropped silently.
