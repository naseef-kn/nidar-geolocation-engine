# NIDAR configuration guide

This guide lists every value that can be configured, where it lives, its unit,
and what it currently is. Use it to know which file to edit when a hardware
measurement arrives.

Status meanings:
- **Set:** a real value from the team or a measurement.
- **Placeholder:** a stand-in value. Do not use results from it for flight.
- **Pending:** not yet measured or decided. Must be filled before flight.

## 1. Fleet (`config/fleet.json`)

| Key | Unit | Current value | Status | Who supplies it |
|---|---|---|---|---|
| `max_total_kg` | kg | 25.0 | Set | Team limit, covers all drones combined |

## 2. Each drone (`config/alpha.json`, `config/bravo.json`)

| Key | Unit | Current value | Status | Who supplies it |
|---|---|---|---|---|
| `drone_id` | text | alpha / bravo | Set | Team naming |
| `capabilities` | list | scout, delivery | Set | Both drones do both roles |
| `telemetry_source` | `simulated` or `mavlink` | simulated | Placeholder | Systems integration, when telemetry is live |
| `camera_calibration_file` | file path | null | Pending | Hardware team, after calibration |
| `airframe_kg` | kg | 0.0 | Pending | Airframe owner, weighed with battery, no packet |
| `packet_kg` | kg | 0.2 | Set | Measured packet weight (200 g) |
| `payload_packets` | count | 0 | Placeholder | Mission planning, how many packets this drone carries |

Rule: the sum of `airframe_kg + payload_packets × packet_kg` over all drones
must be at most the fleet limit. Call `validate_fleet` with all loaded drone
configs and the limit from `load_fleet_limit`; it raises `DroneConfigError`
if the combined limit or unique-ID requirement is violated.

## 3. Camera (currently in code, `nidar/camera_config.py`)

| Parameter | Unit | Current value | Status | Source |
|---|---|---|---|---|
| fx, fy | pixels | 1200.0 | Placeholder | Calibration |
| cx, cy | pixels | 640, 360 | Placeholder | Calibration |
| image width, height | pixels | 1280 × 720 | Placeholder | Flight resolution |
| k1, k2, p1, p2 | dimensionless | 0 | Placeholder | Calibration |
| k3 | dimensionless | not used | Pending | Stored from calibration, not yet in the engine |

Planned: move these to a per-drone camera file, so each drone has its own
calibration. Each drone's `camera_calibration_file` points to it.

## 4. Camera and GNSS mounting (currently in code)

| Parameter | Unit | Current value | Status | Source |
|---|---|---|---|---|
| Camera offset in body frame (x, y, z) | mm | (0, 0, +100) | Placeholder | Hardware measurement |
| Camera-to-body rotation | matrix | Mounting A | Set (approved), not yet verified on hardware | Hardware verification |
| GNSS antenna offset (x, y, z) | mm | not used | Pending | Hardware measurement |
| Camera roll, pitch, yaw | degrees | 0, 0, 0 | Pending | Inclinometer |

## 5. Reference frame and ground (currently in code)

| Parameter | Unit | Current value | Status | Source |
|---|---|---|---|---|
| Origin latitude, longitude | degrees | 0.0, 0.0 default in engine | Pending | Survey reference point |
| Origin altitude | m | 0.0 | Pending | Survey and altitude reference (MSL, ellipsoidal) |
| Ground elevation (`ground_z`) | m | 0.0 (flat ground) | Placeholder | Site survey |

## 6. Target logic (currently in code)

| Parameter | Unit | Current value | Status | Source |
|---|---|---|---|---|
| Reassignment: low confidence below | 0–1 | 0.6 | Placeholder | Field data |
| Reassignment: high confidence at | 0–1 | 0.85 | Placeholder | Field data |
| Reassignment: minimum margin | 0–1 | 0.2 | Placeholder | Field data |
| Alert threshold (wayback high confidence) | 0–1 | uses high confidence above | Placeholder | Field data |
| Duplicate radius | metres | 3.0 | Placeholder | Field data, measured accuracy |
| Range guard: max off-nadir | degrees | 60 | Placeholder | Field data |
| Range guard: medium-confidence angle | degrees | 75 | Placeholder | Field data |

## 7. Not yet configurable

- Telemetry source (MAVLink) for live position and attitude.
- Frame timestamps for stale-detection checks.
- Drop-phase signal from the drop automation.

## Where to edit

1. Fleet limit: `config/fleet.json`.
2. Per-drone values: `config/<drone>.json`.
3. Everything in sections 3 to 6 is still in code; it will move to config files in a later step.
