# SEVAK-X
## Geolocation Engine – Hardware Task Assignment & Data Collection Plan

**Department:** Hardware & Avionics  
**Subsystem:** Geolocation Engine  
**Task Owner:** Naseef – Geolocation Engine & Target Deduplication  
**Supporting Team:** Hardware, Avionics and Flight Operations

---

## 1. Objective

The objective of this assignment is to collect and verify the physical measurements, camera information and flight-controller data required by the SEVAK-X Geolocation Engine.

The geolocation software uses camera images, drone position, altitude and orientation to estimate the geographical coordinates of detected objects.

To make this process reliable, the software requires accurate information about the physical camera setup and the data provided by the onboard hardware.

**The hardware team is responsible for physical measurements, hardware verification and data collection. The software implementation, calibration processing, mathematical calculations and geolocation algorithms will be handled separately by the Geolocation Engine developer.**

No software development is required as part of this assignment.

---

## 2. Scope of Work

The hardware assignment is divided into six major activities.

### Task 1 – Hardware Identification and Documentation

Identify and document the exact hardware being used.

**Components to verify:**
- Camera module and lens.
- Jetson board model (Nano / Orin Nano – verify).
- Flight controller model (confirm from the physical board).
- GPS/GNSS module.
- Camera and flight-controller connection interfaces.

**Information to record:**
- Exact hardware model numbers.
- Camera resolution.
- Lens and focus settings.
- Available communication interfaces.
- Connection details between the camera, Jetson, GPS and flight controller.

**Expected output:** `hardware_inventory.md`

### Task 2 – Camera Calibration Image Collection

**Objective:** Capture photographs of a checkerboard pattern for camera calibration.

**Procedure:**

1. Generate a checkerboard containing 10 × 7 squares (9 × 6 internal corners).
2. Use 25 mm as the intended square size.
3. Print at 100% scale without automatic resizing.
4. Mount the printed pattern on a flat, rigid surface.
5. Physically measure the actual square size and record it.
6. Set the camera to the intended operating resolution and focus.
7. Capture approximately 20–30 photographs from different distances, angles and positions.
8. Ensure that the pattern is clearly visible across different regions of the camera frame.

**Important:** Keep all original photographs unchanged. Separate unclear or unusable photographs rather than deleting the original data.

**Expected output:**
- Original checkerboard photographs.
- Actual measured square size.
- Camera resolution.
- Focus and lens settings.
- Short notes about the capture conditions.

**Software responsibility:** Camera intrinsic and distortion calibration will be calculated and reviewed by the software owner.

### Task 3 – Camera Mounting and Orientation Measurements

**Objective:** Verify the physical camera orientation and measure its position relative to the drone.

**Approved orientation convention:**

**Image top = Drone nose**

The camera orientation must be physically verified before it is accepted by the software.

**Procedure:**

1. Position the drone with its nose facing a clearly marked direction.
2. Capture an image and verify that the marked direction appears towards the top of the image.
3. Rotate the drone by 90 degrees and repeat the check.
4. Capture photographs documenting both tests.
5. Identify a fixed reference point on the drone, such as the flight-controller/IMU reference location.
6. Measure the camera's position relative to that reference point.

**Measurements required:**

| Axis | Direction | Measurement |
|---|---|---|
| X | Forward / backward | mm |
| Y | Right / left | mm |
| Z | Down / up | mm |

Also record the camera's mounting roll, pitch and yaw if suitable measuring equipment is available.

**Expected output:**
- Camera orientation photographs.
- Camera mounting measurements.
- Measurement reference point.
- Measurement method and uncertainty.

### Task 4 – GPS/GNSS Antenna Position Measurements

**Objective:** Determine the physical position of the GPS antenna relative to the same reference point used for the camera measurements.

**Procedure:**

1. Identify the GPS antenna's physical position.
2. Measure its X, Y and Z offsets from the chosen drone reference point.
3. Record all measurements in millimetres.
4. Mention whether the measurement refers to the antenna's geometric centre or a documented antenna reference point.

**Expected output:**
- GPS antenna position measurements.
- Reference point description.
- Measurement method and uncertainty.

### Task 5 – Flight Controller and Telemetry Verification

**Objective:** Verify the availability and format of the information required by the Geolocation Engine.

Check which of the following data can be obtained from the flight controller and GPS module:

| Required information | What to verify |
|---|---|
| GPS latitude | Availability and units |
| GPS longitude | Availability and units |
| Altitude | Value and reference type |
| Roll | Availability, units and sign |
| Pitch | Availability, units and sign |
| Yaw | Availability, units and sign |
| GPS quality | Fix type and satellite count |
| Timestamps | Availability and format |
| Update rate | Frequency of position and attitude data |

Also document the communication interface between the Jetson and flight controller, including the connection type and baud rate where applicable.

Collect sample telemetry logs wherever possible.

**Expected output:**
- Sample GPS data.
- Sample altitude data.
- Sample attitude data.
- Hardware communication details.
- Notes on unavailable or unverified information.

Do not assume that a particular field is available. Record what the hardware actually provides.

### Task 6 – Camera and Telemetry Timestamp Verification

**Objective:** Understand whether camera images and flight-controller telemetry can be matched to the same point in time.

**Procedure:**

1. Identify how image timestamps are generated.
2. Identify how flight-controller telemetry timestamps are generated.
3. Determine whether both systems use compatible time references.
4. If possible, conduct a repeatable test to estimate the time difference.
5. Collect at least 10 observations if the required equipment and logging setup are available.

**Expected output:**
- Sample image timestamps.
- Sample telemetry timestamps.
- Timestamp comparison records.
- Any observed time difference or synchronization limitations.

If this cannot be completed with the current setup, document the limitation and the equipment or access required.

---

## 3. Future Activity – Ground-Truth Data Collection

This activity will be planned after the initial hardware verification and data collection are completed.

**Objective:** Collect real-world reference data to evaluate the accuracy of the Geolocation Engine.

Proposed activities:

1. Identify at least four visible ground markers.
2. Obtain their known geographical coordinates using an appropriate surveyed reference.
3. Record the method and accuracy of the reference measurements.
4. Capture images of the markers using the drone camera.
5. Collect corresponding GPS, altitude and attitude telemetry.
6. Record the images and telemetry together for software-side testing.

Flight testing must be coordinated with the Flight Operations Lead and performed only after appropriate safety checks and permissions.

**Expected output:** Ground-marker coordinates, original photographs and corresponding telemetry data.

---

## 4. Final Data Handover Structure

All collected information should be organised into the following folder structure:

```text
hardware_handover/
│
├── camera/
│   ├── calibration_images/
│   ├── camera_details.md
│   └── capture_notes.md
│
├── measurements/
│   ├── camera_mounting_measurements.md
│   ├── camera_orientation.md
│   └── gps_antenna_measurements.md
│
├── telemetry/
│   ├── sample_gps.csv
│   ├── sample_attitude.csv
│   ├── sample_altitude.csv
│   └── timestamp_records.csv
│
├── ground_truth/
│   ├── reference_coordinates.csv
│   └── captured_images/
│
└── documentation/
    ├── hardware_inventory.md
    └── known_limitations.md
```

Files for tasks that are not yet completed can be added later. Do not create fabricated measurements or placeholder results and present them as completed work.

---

## 5. Responsibilities

| Hardware Team | Geolocation Software Owner |
|---|---|
| Identify physical hardware | Review hardware specifications |
| Print and measure checkerboard | Process calibration photographs |
| Capture calibration images | Calculate camera calibration parameters |
| Verify camera orientation | Implement coordinate transformations |
| Measure camera and GPS offsets | Integrate measurements into software |
| Verify GPS and telemetry availability | Implement telemetry processing |
| Collect timestamp records | Implement timestamp alignment |
| Collect ground-truth data | Evaluate geolocation performance |
| Document hardware limitations | Maintain and test geolocation modules |

---

## 6. Important Guidelines

1. Every measurement must include its unit.
2. Every physical offset must specify its reference point.
3. Do not estimate or invent missing hardware specifications.
4. Preserve original photographs and raw telemetry data.
5. Record unknown or unavailable information clearly.
6. Camera orientation must be physically verified rather than assumed.
7. PTZ measurements are required only if the camera's pan, tilt or zoom functions will be used during operation.
8. Ground-truth flight experiments must be coordinated with the Flight Operations Lead.
9. Hardware members are not required to modify the geolocation source code.

---

## 7. Execution Priority

| Priority | Activity | Expected completion |
|---|---|---|
| 1 | Hardware identification | First |
| 2 | Checkerboard preparation and image collection | After hardware identification |
| 3 | Camera orientation and mounting measurements | Initial physical setup |
| 4 | GPS antenna measurements | Initial physical setup |
| 5 | Telemetry verification | After connections are available |
| 6 | Timestamp verification | After image and telemetry logging are available |
| 7 | Ground-truth experiments | Later validation stage |

**Note:** The activities may overlap depending on hardware availability and team allocation.

---

## 8. Completion Criteria

The initial hardware assignment will be considered complete when:

- Exact hardware models have been documented.
- Original checkerboard photographs have been collected.
- Actual checkerboard square dimensions have been measured.
- Camera orientation has been physically verified.
- Camera and GPS antenna offsets have been measured.
- Available GPS, altitude and attitude data have been documented.
- Sample telemetry data has been provided wherever available.
- Timestamp availability and synchronization limitations have been documented.
- All collected files have been organised and handed over.

**Final Objective:** Provide the Geolocation Engine developer with verified physical measurements, camera images and hardware data required to continue software development and real-world validation of the SEVAK-X geolocation system.

