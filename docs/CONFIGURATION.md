# NIDAR configuration guide

Lists every configurable value, the file it lives in, its unit, and its status.

Status meanings:
- **Set:** a real, agreed value.
- **Placeholder:** a stand-in. Do not use results from it for flight.
- **Pending:** not yet measured or decided. Must be filled before flight.
- **Code only:** not in a file on purpose. Changing it requires a design change.

## 1. Fleet (`config/fleet.json`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `max_total_kg` | kg | 25.0 | Set | Team limit, covers all drones combined |

## 2. Each drone (`config/alpha.json`, `config/bravo.json`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `drone_id` | text | alpha / bravo | Set | Team naming |
| `capabilities` | list | scout, delivery | Set | Both drones do both roles |
| `telemetry_source` | `simulated` or `mavlink` | simulated | Placeholder | Systems integration, once telemetry is live |
| `camera_calibration_file` | file path | null | Pending | Hardware team, after calibration |
| `airframe_kg` | kg | 0.0 | Pending | Airframe owner, weighed with battery and no packet |
| `packet_kg` | kg | 0.2 | Set | Measured packet weight (200 g) |
| `payload_packets` | count | 0 | Placeholder | Mission planning |

Rule: the sum of `airframe_kg + payload_packets × packet_kg` over all drones must be at most `max_total_kg`. The loader checks this.

## 3. Camera (`config/camera.json`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `image_width`, `image_height` | pixels | 1280 × 720 | Placeholder | Flight resolution, hardware team |
| `fx`, `fy` | pixels | 1200.0 | Placeholder | Calibration output |
| `cx`, `cy` | pixels | 640.0, 360.0 | Placeholder | Calibration output |
| `offset_cam_in_body_m` | metres, body FRD | [0, 0, 0.1] | Placeholder | Hardware measurement from the reference point |
| `distortion_enabled` | true / false | false | Pending | Software, after distortion correction is implemented |
| `k1`, `k2`, `p1`, `p2` | dimensionless | 0.0 | Placeholder | Calibration output |

`distortion_enabled` and the `k`/`p` values have no effect on results yet. Distortion correction is not implemented.

Code only: the camera-to-body mounting matrix (Mounting A), set in `nidar/camera_config.py`.

## 4. Range guard (`config/range_guard.json`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `enabled` | true / false | true | Set | Software |
| `max_off_nadir_deg` | degrees | 60.0 | Placeholder | Field data |
| `medium_confidence_off_nadir_deg` | degrees | 75.0 | Placeholder | Field data |
| `action_on_exceed_max_off_nadir` | `reject`, `flag_low`, or `warn` | `flag_low` | Placeholder | Mission policy decision |

Angles are entered in degrees. The loader converts them to radians. A target beyond the max angle with `flag_low` is reported with medium confidence (×0.7), not rejected.

## 5. Reassignment policy (`config/target_logic.json`, `reassignment`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `low_confidence_below` | 0–1 | 0.6 | Placeholder | Field data |
| `high_confidence_at` | 0–1 | 0.85 | Placeholder | Field data |
| `min_margin` | 0–1 | 0.2 | Placeholder | Field data |

These values are used by the coordinator's wayback alert rule (`high_confidence_at`). `low_confidence_below` and `min_margin` apply to the reassignment function, which the coordinator no longer calls automatically.

## 6. Cross-frame deduplication (`config/target_logic.json`, `crossframe`)

| Key | Unit | Current value | Status | Supplied by |
|---|---|---|---|---|
| `duplicate_radius_m` | metres | 3.0 | Placeholder, unmeasured | Ground-truth test, after geolocation accuracy is measured |

The loader validates this value at load time, so a zero, negative, or non-finite radius fails before any run. The matching code uses this radius for distance-only matching. The time window is not implemented yet and waits on frame timestamps.

## 7. Per-drone pixel deduplication (code only)

| Parameter | Unit | Current value | Status | Notes |
|---|---|---|---|---|
| `spatial_threshold_pixels` | pixels | 50.0 | Placeholder | Applied to one drone's own frame detections. Not in a file. |

## 8. Not yet configurable

- Telemetry source for live position and attitude (MAVLink).
- Frame timestamps for stale-detection checks.
- Drop-phase signal from drop automation.

## Where to edit

1. Fleet limit: `config/fleet.json`.
2. Per-drone values: `config/alpha.json`, `config/bravo.json`.
3. Camera values: `config/camera.json`.
4. Range guard: `config/range_guard.json`.
5. Reassignment policy and cross-frame radius: `config/target_logic.json`.
6. Per-drone pixel threshold: code only, in `nidar/deduplicator.py`.

See `docs/KNOWN_LIMITATIONS.md` for what the software does not yet guarantee.
