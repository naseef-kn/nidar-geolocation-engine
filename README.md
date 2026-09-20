# NIDAR: Autonomous Flood-Rescue Drone System

**A multi-drone geolocation and task coordination engine for autonomous flood-rescue operations.**

NIDAR enables a swarm of autonomous drones to detect survivors in flooded areas, geolocate them with high accuracy, and coordinate delivery of rescue supplies using a distributed task management system.

---

## Project Overview

NIDAR stands for **N**avigation and **I**mage **D**etection for **A**utonomous **R**escue. The system is designed to be simulation-first, with mathematical rigor and comprehensive testing at every phase.

### Key Features

- **Accurate geolocation** — from camera pixel → GPS coordinates via 3D transforms
- **Attitude-aware** — handles drone roll, pitch, yaw in real-time
- **Confidence scoring** — off-nadir angle validation with dynamic confidence multiplier
- **Task coordination** — Scout drones detect, delivery drones execute
- **Modular architecture** — each phase is independently testable

### Timeline

- **September 2026** — Phase 1 & 2A complete (78 tests passing)
- **October 2026** — Phase 2B (real camera + flight controller integration)
- **November 2026** — Live field testing & performance benchmarking

---

## Development Status

### ✅ Phase 1: MVP Core (23 tests) — COMPLETE

**Objective:** Proof-of-concept end-to-end flow with one scout and one delivery drone.

**Completed:**
- \`detection.py\` — Pixel-level survivor detection with confidence
- \`scout_drone.py\` — Scout drone autonomy (detect, report)
- \`target.py\` — Survivor target representation with ID tracking
- \`deduplicator.py\` — Merge duplicate detections within spatial threshold
- \`delivery_task.py\` — Task state machine (pending → assigned → completed)
- \`delivery_drone.py\` — Delivery drone autonomy (accept task, execute)
- \`__init__.py\` — Full package exports

**Mathematical foundation:** None yet (direct pixel coordinates).

**Tests:** 23 passing, covering all state transitions and edge cases.

---

### ✅ Phase 2A: Geolocation Engine (50+ tests) — COMPLETE

**Objective:** Transform pixel detections into world-frame GPS coordinates with validation.

**Completed:**

#### 1. **Camera Model** (\`camera_config.py\`)
- Intrinsic calibration: focal lengths (fx, fy), principal point (cx, cy)
- **Mounting A geometry:** camera rotated relative to drone body
  - Camera X-axis (image right) → Body Y-axis (starboard)
  - Camera Y-axis (image down) → Body X-axis (nose forward)
  - Camera Z-axis (optical axis) → Body Z-axis (down)
- Camera offset in body frame: 10 cm below body center
- Validation: orthonormality, determinant, principal point bounds
- Synthetic config: 1280×720 image, fx=fy=1200, cx=640, cy=360

#### 2. **3D Transforms** (\`transforms.py\`)
- Euler angle → rotation matrix (ZYX convention: yaw, pitch, roll)
- NED (North-East-Down) frame coordinate system
- Geographic conversion (NED ↔ lat/lon/altitude)
  - Uses WGS84 ellipsoid (R_earth = 6,371 km)
  - Local origin: 12.9716°N, 77.5946°E, 502m MSL (Bangalore)

**Math:**
\`\`\`
R_world_body = R_z(yaw) @ R_y(pitch) @ R_x(roll)
ray_world = R_world_body @ ray_body
ground_ned = camera_origin + λ * ray_world   (λ = intersection parameter)
lat, lon = geographic_from_ned(ground_ned)
\`\`\`

#### 3. **Camera Geometry** (\`camera_geometry.py\`)
- Pixel → normalized camera ray
- \`ray_cam = [(u - cx)/fx, (v - cy)/fy, 1]\` (normalized, z=1)
- Handles full image dimensions (1280×720 tested)

#### 4. **Geolocation Pipeline** (\`geolocation.py\`)
- **Input:** PixelDetection (u, v, confidence) + DroneState (position, attitude) + CameraConfig
- **Processing:**
  1. Pixel → camera ray (normalized 3D vector)
  2. Apply camera-to-body rotation (Mounting A)
  3. Apply body-to-world rotation (drone attitude)
  4. Find intersection with ground plane (z=0)
  5. Convert NED → lat/lon/altitude
  6. Compute slant range (distance from camera to ground point)
- **Output:** GeolocatedTarget (lat, lon, alt, confidence, slant_range, off_nadir_angle)
- **Error handling:**
  - TelemetryError: drone below ground elevation
  - NoIntersectionError: ray parallel to ground
  - All errors caught with helpful messages

#### 5. **Range Guard Validation** (\`range_guard.py\`)
- **Purpose:** Reject low-confidence detections from extreme angles
- **Validation metric:** off-nadir angle = arccos(|ray_world_z|)
- **Rules:**
  - 0°–30°: HIGH confidence (nadir region, trusted)
  - 30°–60°: MEDIUM confidence (oblique, acceptable)
  - 60°–90°: FLAG_LOW_CONFIDENCE (horizon, uncertain)
  - 90°: REJECTED (sky, invalid)
- **Altitude-independent:** works at any drone altitude
- **Default threshold:** 60° max off-nadir angle

#### 6. **Target Output** (\`geolocated_target.py\`)
- GeolocatedTarget dataclass: lat, lon, alt, confidence, slant_range, off_nadir_angle
- GeolocationDebugInfo: intermediate values (ray, origin, intersection) for analysis
- Confidence multiplier function: scales based on off-nadir angle

---

### 🚀 Phase 2B: Real-World Integration — NEXT

**Objective:** Move from synthetic simulation to real flight data.

**Planned:**
- [ ] Real camera calibration (OpenCV charuco board or DJI specs)
- [ ] PX4/Pixhawk telemetry integration (read live attitude + GPS)
- [ ] Range guard tuning (empirical confidence thresholds from flights)
- [ ] Video frame processing (live pixel detection + geolocation per frame)
- [ ] Performance benchmarking (accuracy vs. ground truth, latency per frame)
- [ ] Field testing in controlled environment (parking lot accuracy validation)

---

## Installation

### Prerequisites
- Python 3.14+
- macOS, Linux, or Windows
- Virtual environment (recommended)

### Setup

\`\`\`bash
# Clone repo
git clone https://github.com/naseef-kn/nidar-geolocation-engine.git
cd nidar-geolocation-engine

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate  # On Windows: .venv\\Scripts\\activate

# Install dependencies
pip install numpy pytest

# Run tests
python -m pytest tests/ -v
\`\`\`

**Expected result:** 78 tests passing in ~0.1 seconds.

---

## Architecture

### Directory Structure

\`\`\`
nidar-geolocation-engine/
├── nidar/                          # Main package
│   ├── __init__.py                 # Exports all public classes/functions
│   │
│   ├── Phase 1: Detection & Delivery
│   ├── detection.py                # PixelDetection dataclass + validation
│   ├── scout_drone.py              # Scout autonomy
│   ├── target.py                   # Survivor target (ID, location, confidence)
│   ├── deduplicator.py             # Merge nearby detections
│   ├── delivery_task.py            # Task state machine
│   ├── delivery_drone.py            # Delivery drone autonomy
│   │
│   └── Phase 2A: Geolocation Engine
│       ├── camera_config.py        # CameraConfig + Mounting A
│       ├── camera_geometry.py      # pixel_to_camera_ray()
│       ├── transforms.py           # Euler angles, NED/geographic conversion
│       ├── geolocation.py          # Main pipeline (compute_geolocation)
│       ├── range_guard.py          # Off-nadir validation + confidence
│       └── geolocated_target.py    # GeolocatedTarget output + helpers
│
├── tests/
│   ├── Phase 1 tests (23 total)
│   │   ├── test_detection.py
│   │   ├── test_scout_drone.py
│   │   ├── test_target.py
│   │   ├── test_deduplicator.py
│   │   ├── test_delivery_task.py
│   │   ├── test_delivery_drone.py
│   │   ├── test_mvp_end_to_end.py
│   │   └── test_hello_nidar.py
│   │
│   └── Phase 2A tests (50+ total)
│       ├── test_camera_config.py      # Intrinsics, Mounting A, validation
│       ├── test_geolocation.py        # T1-T17: core pipeline + edge cases
│       └── test_range_guard.py        # Off-nadir angles, confidence
│
├── README.md
├── .gitignore
└── .venv/                          # Python virtual environment
\`\`\`

---

## Core Algorithm: The Geolocation Pipeline

### Mathematical Chain (§G Audit ✓)

**Input:**
- PixelDetection: (u, v) pixel coordinates, confidence score
- CameraConfig: intrinsics, Mounting A rotation, offset
- DroneState: position (north, east, down), attitude (roll, pitch, yaw)
- Ground plane: z = 0 (NED)

**Processing:**

\`\`\`
Step 1: Normalize pixel to camera ray
  ray_cam = [(u - cx) / fx, (v - cy) / fy, 1]
  ray_cam_norm = normalize(ray_cam)

Step 2: Apply camera-to-body rotation (Mounting A)
  ray_body = R_cam_to_body @ ray_cam_norm

Step 3: Apply body-to-world rotation (drone attitude)
  R_world_body = R_z(yaw) @ R_y(pitch) @ R_x(roll)
  ray_world = R_world_body @ ray_body

Step 4: Find ground intersection
  camera_origin_world = drone_position + R_world_body @ offset_cam_in_body
  λ = (z_ground - camera_origin_world.z) / ray_world.z
  ground_ned = camera_origin_world + λ * ray_world

Step 5: Convert NED → geographic
  (lat, lon, alt) = ned_to_geographic(ground_ned, origin=(12.9716°N, 77.5946°E))

Step 6: Compute off-nadir angle
  off_nadir = arccos(|ray_world.z|)  [in degrees]

Step 7: Validate with range guard
  if off_nadir ≤ 30°:  confidence = HIGH
  if 30° < off_nadir ≤ 60°:  confidence = MEDIUM
  if 60° < off_nadir < 90°:  confidence = FLAG_LOW
  if off_nadir ≥ 90°:  REJECTED (sky ray)
\`\`\`

**Output:**
- GeolocatedTarget with lat, lon, altitude, confidence, slant_range, validation_result

### Example: Nadir Pixel (T1)

\`\`\`
Input:
  - Pixel: (640, 360) [center of 1280×720 image]
  - Drone: z = -50m, level attitude (roll=pitch=yaw=0)

Execution:
  ray_cam = [0, 0, 1]  [looking straight down]
  ray_body = [0, 0, 1]
  ray_world = [0, 0, 1]  [no rotation]
  camera_origin = (0, 0, -49.9)  [50m altitude - 0.1m offset]
  λ = (0 - (-49.9)) / 1 = 49.9
  ground_ned = (0, 0, 0)  [directly below]
  off_nadir = 0°
  slant_range = 49.9m (exactly)

Output:
  GeolocatedTarget(
    lat=12.9716°N,
    lon=77.5946°E,
    alt=502m,
    confidence=HIGH,
    slant_range_m=49.9,
    off_nadir_deg=0.0
  )
\`\`\`

---

## Testing

### Test Coverage (78 total)

#### Phase 1 (23 tests)
- Detection pipeline: valid detections, boundary rejection, confidence validation
- Scout drone: detection flow, invalid frame dimensions
- Target: valid/invalid confidence, ID validation
- Deduplicator: single target, multiple targets, merging logic
- Delivery task: state machine (pending → assigned → completed)
- Delivery drone: task acceptance, invalid scenarios
- End-to-end: full MVP flow

#### Phase 2A Core (17 tests: T1-T8, T10-T17)
- **T1:** Centre pixel (nadir) → exact 49.9m slant range
- **T2:** Corner pixel (southeast) → correct lat/lon offset
- **T3:** Camera offset invariance → different z offsets produce correct slant range delta
- **T4:** Zero attitude → identity rotation (no effect)
- **T5:** Yaw 90° → correct frame rotation
- **T6:** Pitch up → centre pixel geolocation still valid
- **T7:** Nadir slant range exactly 49.9m (no floating-point error)
- **T8:** Radian-to-degree conversion (π/2 → 90°)
- **T10:** Sky ray (90° off-nadir) → REJECTED
- **T11:** Horizon ray (85° off-nadir) → REJECTED
- **T12:** Camera below ground → TelemetryError
- **T13:** Rotation invariants (determinant=1, orthonormal)
- **T14:** Unit norm preservation (ray length invariant)
- **T17:** Round-trip geographic conversion (NED → lat/lon → NED)

#### Range Guard (10 tests)
- Off-nadir angles: 0°, 30°, 45°, 60°, 80° → correct confidence levels
- Guard disabled → accepts all angles
- Parallel ray (0° vertical component) → REJECTED
- Negative slant range → REJECTED
- Camera below ground → REJECTED

#### End-to-End (5 tests)
- Full pipeline integration
- Phase 1 + Phase 2A combined flow
- Configuration initialization

### Running Tests

\`\`\`bash
# All tests with verbose output
python -m pytest tests/ -v

# Specific test file
python -m pytest tests/test_geolocation.py -v

# Specific test
python -m pytest tests/test_geolocation.py::TestCore::test_T1_centre_pixel_nadir -v

# With print statements visible
python -m pytest tests/ -v -s
\`\`\`

**Expected:** 78 passed in ~0.1s

---

## Key Parameters (Synthetic Configuration)

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Image size** | 1280×720 | HD resolution |
| **Focal length (fx, fy)** | 1200 | Square pixels, ~53° HFOV |
| **Principal point (cx, cy)** | 640, 360 | Center of image |
| **Camera offset** | (0, 0, 0.1) m | 10 cm below drone body |
| **Camera mount** | Mounting A | See rotation matrix above |
| **Ground plane** | z = 0 NED | Sea level equivalent |
| **Local origin (lat, lon)** | 12.9716°N, 77.5946°E | Bangalore, India |
| **Drone test altitude** | -50m NED | 50m above ground |
| **Max off-nadir angle** | 60° | Range guard threshold |
| **Earth radius** | 6,371 km | WGS84 approximation |

---

## Roadmap

### Phase 2B (October 2026)
- [ ] Integrate real camera calibration (charuco board validation)
- [ ] PX4 telemetry streaming (MAVLink protocol)
- [ ] Live video frame loop (1920×1080 processing)
- [ ] Latency profiling (target: <50ms per frame)
- [ ] Field test (controlled parking lot scenario)

### Phase 3 (November 2026)
- [ ] Multi-drone coordination (3+ scouts + 2+ deliveries)
- [ ] Survival index prediction (based on geolocation confidence)
- [ ] Route optimization (shortest path for delivery drones)
- [ ] Swarm autonomy (decentralized task assignment)

### Phase 4 (December 2026)
- [ ] Real-world field testing (approved test site, full swarm)
- [ ] Sensor fusion (camera + LiDAR + sonar)
- [ ] Adaptive confidence thresholds (machine learning)

---

## Quick Start

\`\`\`bash
# 1. Clone & setup
git clone https://github.com/naseef-kn/nidar-geolocation-engine.git
cd nidar-geolocation-engine
python3 -m venv .venv
source .venv/bin/activate

# 2. Install
pip install numpy pytest

# 3. Test
python -m pytest tests/ -v

# 4. Import in your code
from nidar import (
    PixelDetection, ScoutDrone, Target, TargetDeduplicator,
    DeliveryTask, DeliveryDrone,
    CameraConfig, SYNTHETIC_CAMERA_CONFIG, 
    DroneState, compute_geolocation
)
\`\`\`

---

## Contributing

NIDAR is a student research project. All development is in \`main\` branch.

**Commit convention:**
- \`feat: <description>\` — new feature (Phase 2A geolocation engine)
- \`fix: <description>\` — bug fix
- \`docs: <description>\` — documentation update
- \`test: <description>\` — new tests

**Test before push:**
\`\`\`bash
python -m pytest tests/ -v
\`\`\`

All commits must have passing tests.

---

## Author

**Naseef** — BTech CSE, 3rd semester  
Bangalore Institute of Technology (BIT)

NIDAR is part of a broader autonomous systems research portfolio exploring drone swarm coordination, computer vision, and embedded systems.

---

## License

MIT (or CC0 for educational use)

---

## Quick Links

- **Repo:** https://github.com/naseef-kn/nidar-geolocation-engine
- **Latest commit:** \`e044bac\` (Phase 2A complete)
- **Tests:** 78 passing, 2950 insertions, 9 core modules
