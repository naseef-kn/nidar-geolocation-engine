# Known limitations (software)

This lists what the geolocation and target software does not yet guarantee.
Each item states its current status and what would close it.

## Security and trust

- **Transport trusts the sender field.** `InMemoryTransport` does not authenticate `message.sender`. This is acceptable in one process. A radio or network link needs message authentication before deployment.
- **Payload counts are trusted.** The coordinator accepts a drone's reported `payload_remaining` without a plausibility check against its configured capacity. A faulty or malicious drone could claim more packets than it carries.

## Accuracy and measurement

- **Geolocation accuracy is unmeasured.** Results use synthetic camera values (fx = fy = 1200, cx = 640, cy = 360) and a flat-ground assumption. No accuracy claim applies until the ground-truth test runs.
- **50 ms per-frame timing is not measured.** The target is recorded but not verified on the Jetson hardware. Measure before claiming it.
- **No distortion correction.** `distortion_enabled` is false, and the `k` and `p` values have no effect until correction is implemented.
- **Duplicate radius is a placeholder.** `duplicate_radius_m = 3.0` is not tied to measured accuracy.

## Deduplication

- **No time window in cross-frame matching.** Matching uses distance only. Two separate survivors within the radius are merged, and a survivor that moves is not tracked. The time window waits on frame timestamps.
- **No cross-drone matching in the coordinator.** Two drones reporting the same survivor create two targets. Only per-drone pixel deduplication exists, and it is not yet wired into the drone pipeline.
- **Pixel deduplication is not in the reporting path.** A repeated detection in consecutive frames is reported each time until the pipeline is wired.

## Memory and scale

- **Registry and ID factory grow without bound.** Targets, alerts, and rejected messages are kept in memory for the whole mission. A long mission with many detections increases memory use. A cap or cleanup policy is needed.
- **Cross-frame matching is linear.** Each new target is compared against every known target. Fine for tens of survivors, not for thousands.

## Missing inputs

- **No live telemetry.** `DroneState` is built by hand. The MAVLink source is not connected.
- **No frame timestamps.** Stale detections cannot be rejected yet.
- **No drop-phase signal.** A `DROPPING` status is never set in a real flight. Drop protection is tested in software only.
- **Calibration is synthetic.** See the accuracy section.

## Ownership gaps

- **Systems Integration, Simulation and Flight Operations, and Autopilot/Drop Automation roles are unassigned.** The items above depend on them.
