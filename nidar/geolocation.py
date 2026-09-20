"""
NIDAR Geolocation Engine — Phase 2A

Complete pipeline: pixel → geographic coordinates with validation.

Pipeline steps (§G from audit):
  1. Pixel → camera ray (using intrinsics)
  2. Camera ray → body ray (using mounting rotation)
  3. Body ray → world ray (using drone attitude)
  4. Camera origin in world frame (position + mounting offset)
  5. Ray-plane intersection (ground at z = 0)
  6. Ground point in NED → geographic (lat/lon/alt)
  7. Range guard validation (off-nadir angle check)
  8. Output: GeolocatedTarget

Invariants:
  - All rays are unit-norm
  - All rotations are proper (det = +1)
  - Ground plane is defined by z = ground_z (NED convention)
  - Altitude-independent guard (off-nadir angle, not slant range)

This engine is pure geometry. No hard-coded camera parameters.
All parameters come from CameraConfig and RangeGuardConfig.
"""

import numpy as np
from typing import Tuple, Optional

from .camera_config import CameraConfig
from .camera_geometry import pixel_to_camera_ray
from .transforms import (
    euler_to_rotation,
    ned_to_geographic,
    is_unit_norm,
    is_proper_rotation,
)
from .range_guard import (
    RangeGuardConfig,
    validate_geolocation,
    ValidationResult,
)
from .geolocated_target import (
    GeolocatedTarget,
    GeolocationDebugInfo,
    combine_confidence_levels,
)


# ============================================================================
# DATA CLASSES FOR INTERMEDIATE RESULTS
# ============================================================================

class GeolocationError(Exception):
    """Base exception for geolocation errors."""
    pass


class TelemetryError(GeolocationError):
    """Error in drone telemetry (e.g., camera below ground)."""
    pass


class NoIntersectionError(GeolocationError):
    """Ray does not intersect ground plane (or intersects behind camera)."""
    pass


class ValidationError(GeolocationError):
    """Geolocation result fails validation checks."""
    pass


# ============================================================================
# DRONE STATE STRUCTURE
# ============================================================================

class DroneState:
    """
    Drone telemetry and state (immutable).
    
    Attributes:
        position_ned_m: (x, y, z) position in NED frame (metres)
                        z < 0 means above ground (since +z = down)
        roll_rad: Roll angle (radians, positive = right wing down)
        pitch_rad: Pitch angle (radians, positive = nose up)
        yaw_rad: Yaw angle (radians, positive = nose rotates toward east in top-down view)
    """
    
    def __init__(
        self,
        position_ned_m: Tuple[float, float, float],
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
    ):
        self.position_ned_m = np.array(position_ned_m, dtype=np.float64)
        self.roll_rad = float(roll_rad)
        self.pitch_rad = float(pitch_rad)
        self.yaw_rad = float(yaw_rad)
        
        # Validate position is above ground (z < 0 in NED)
        if self.position_ned_m[2] >= 0:
            raise ValueError(
                f"Drone z-position {self.position_ned_m[2]} >= 0 (above-ground threshold). "
                f"In NED, z < 0 means above ground."
            )
    
    def rotation_body_to_world(self) -> np.ndarray:
        """Compute body-to-world rotation matrix from Euler angles."""
        return euler_to_rotation(self.roll_rad, self.pitch_rad, self.yaw_rad)


# ============================================================================
# CORE GEOLOCATION FUNCTION
# ============================================================================

def compute_geolocation(
    pixel_u: float,
    pixel_v: float,
    pixel_confidence: float,
    camera_config: CameraConfig,
    drone_state: DroneState,
    ground_z: float = 0.0,
    origin_latitude_deg: float = 0.0,
    origin_longitude_deg: float = 0.0,
    origin_altitude_m: float = 0.0,
    range_guard_config: Optional[RangeGuardConfig] = None,
    return_debug_info: bool = False,
) -> Tuple[GeolocatedTarget, Optional[GeolocationDebugInfo]]:
    """
    Compute geographic coordinates for a pixel detection.
    
    Complete pipeline with full error checking and range guard validation.
    
    Args:
        pixel_u: Pixel x-coordinate (0 ≤ u < image_width)
        pixel_v: Pixel y-coordinate (0 ≤ v < image_height)
        pixel_confidence: Detection confidence from scout drone (0–1)
        camera_config: CameraConfig with intrinsics and mounting
        drone_state: DroneState with position and attitude
        ground_z: Ground elevation in NED frame (default 0, in metres)
        origin_latitude_deg: Local origin latitude (degrees)
        origin_longitude_deg: Local origin longitude (degrees)
        origin_altitude_m: Local origin altitude (metres WGS84)
        range_guard_config: RangeGuardConfig for validation (default: no guard)
        return_debug_info: If True, return debug info (default: False)
    
    Returns:
        (GeolocatedTarget, debug_info_or_None)
    
    Raises:
        ValueError: If pixel is out of bounds
        TelemetryError: If drone state is invalid
        NoIntersectionError: If ray doesn't intersect ground
        ValidationError: If validation fails (depends on config)
    
    Notes:
        This function is pure; no side effects.
        All parameters are inputs; nothing is modified.
    """
    
    # ========================================================================
    # STEP 1: PIXEL → CAMERA RAY (using intrinsics)
    # ========================================================================
    
    try:
        ray_cam = pixel_to_camera_ray(pixel_u, pixel_v, camera_config)
    except ValueError as e:
        raise ValueError(f"Pixel conversion failed: {e}")
    
    assert is_unit_norm(ray_cam), "ray_cam should be unit norm"
    assert ray_cam[2] > 0, "ray_cam.z should be positive (forward along optical axis)"
    
    # ========================================================================
    # STEP 2: CAMERA RAY → BODY RAY (using mounting rotation)
    # ========================================================================
    
    ray_body = camera_config.R_cam_to_body @ ray_cam
    
    assert is_unit_norm(ray_body), "ray_body should be unit norm"
    
    # ========================================================================
    # STEP 3: BODY RAY → WORLD RAY (using drone attitude)
    # ========================================================================
    
    R_body_to_world = drone_state.rotation_body_to_world()
    assert is_proper_rotation(R_body_to_world), "R_body_to_world should be proper rotation"
    
    ray_world = R_body_to_world @ ray_body
    
    assert is_unit_norm(ray_world), "ray_world should be unit norm"
    
    # ========================================================================
    # STEP 4: CAMERA ORIGIN IN WORLD FRAME
    # ========================================================================
    
    # Offset is given in body frame; transform to world frame
    offset_in_body = np.array(camera_config.offset_cam_in_body, dtype=np.float64)
    offset_in_world = R_body_to_world @ offset_in_body
    
    camera_origin_world = drone_state.position_ned_m + offset_in_world
    
    # ========================================================================
    # STEP 5: GROUND INTERSECTION (ray-plane intersection)
    # ========================================================================
    
    # Ray equation: P(λ) = camera_origin + λ * ray_world
    # Ground plane: z = ground_z
    # Solve: camera_origin.z + λ * ray_world.z = ground_z
    #        λ = (ground_z - camera_origin.z) / ray_world.z
    
    # Check camera is above ground
    if camera_origin_world[2] >= ground_z:
        raise TelemetryError(
            f"Camera z-position ({camera_origin_world[2]:.3f}) >= ground z ({ground_z:.3f}). "
            f"Camera must be above ground."
        )
    
    # Check ray is not parallel to ground
    if abs(ray_world[2]) < 1e-6:
        raise NoIntersectionError(
            f"Ray z-component is near zero ({ray_world[2]:.6f}). "
            f"Ray is nearly parallel to ground plane."
        )
    
    # Compute slant range
    slant_range_m = (ground_z - camera_origin_world[2]) / ray_world[2]
    
    # Check slant range is positive (ground in front of camera)
    if slant_range_m <= 0:
        raise NoIntersectionError(
            f"Slant range is non-positive ({slant_range_m:.3f} m). "
            f"Ground plane is behind the camera."
        )
    
    # Ground point in NED frame
    ground_ned = camera_origin_world + slant_range_m * ray_world
    
    # ========================================================================
    # STEP 6: NED → GEOGRAPHIC CONVERSION
    # ========================================================================
    
    # Extract NED coordinates
    north_m = ground_ned[0]
    east_m = ground_ned[1]
    down_m = ground_ned[2]
    
    latitude_deg, longitude_deg, altitude_m = ned_to_geographic(
        north_m=north_m,
        east_m=east_m,
        down_m=down_m,
        origin_latitude_deg=origin_latitude_deg,
        origin_longitude_deg=origin_longitude_deg,
        origin_altitude_m=origin_altitude_m,
    )
    
    # ========================================================================
    # STEP 7: RANGE GUARD VALIDATION
    # ========================================================================
    
    if range_guard_config is None:
        range_guard_config = RangeGuardConfig()  # Default config
    
    validation_result = validate_geolocation(
        slant_range_m=slant_range_m,
        ray_world=ray_world,
        camera_origin_world=camera_origin_world,
        ground_z=ground_z,
        guard_config=range_guard_config,
    )
    
    # Compute final confidence
    final_confidence = combine_confidence_levels(
        pixel_confidence=pixel_confidence,
        validation_confidence=validation_result.confidence_level,
    )
    
    # ========================================================================
    # STEP 8: BUILD OUTPUT
    # ========================================================================
    
    geolocated_target = GeolocatedTarget(
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        confidence=final_confidence,
        pixel_u=pixel_u,
        pixel_v=pixel_v,
        pixel_confidence=pixel_confidence,
        slant_range_m=slant_range_m,
        off_nadir_angle_deg=validation_result.off_nadir_angle_deg or 0.0,
        validation_confidence=validation_result.confidence_level,
        ray_world=ray_world.copy(),
        camera_origin_world=camera_origin_world.copy(),
    )
    
    debug_info = None
    if return_debug_info:
        debug_info = GeolocationDebugInfo(
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            drone_position_ned_m=tuple(drone_state.position_ned_m),
            drone_attitude_rad=(drone_state.roll_rad, drone_state.pitch_rad, drone_state.yaw_rad),
            ray_camera_frame=ray_cam.copy(),
            ray_body_frame=ray_body.copy(),
            ray_world_frame=ray_world.copy(),
            camera_origin_world=camera_origin_world.copy(),
            slant_range_m=slant_range_m,
            ground_position_ned_m=tuple(ground_ned),
            latitude_deg=latitude_deg,
            longitude_deg=longitude_deg,
            altitude_m=altitude_m,
            off_nadir_angle_deg=validation_result.off_nadir_angle_deg or 0.0,
            validation_result_reason=validation_result.reason,
        )
    
    return (geolocated_target, debug_info)


# ============================================================================
# CONVENIENCE WRAPPERS
# ============================================================================

def compute_geolocation_simple(
    pixel_u: float,
    pixel_v: float,
    pixel_confidence: float,
    camera_config: CameraConfig,
    drone_state: DroneState,
    range_guard_config: Optional[RangeGuardConfig] = None,
) -> GeolocatedTarget:
    """
    Simplified geolocation call (no debug info, uses default config).
    
    Assumes:
    - ground_z = 0
    - origin = (0°, 0°, 0 m)
    
    Args:
        pixel_u, pixel_v: Pixel coordinates
        pixel_confidence: Detection confidence (0–1)
        camera_config: Camera intrinsics and mounting
        drone_state: Drone position and attitude
        range_guard_config: Optional range guard config
    
    Returns:
        GeolocatedTarget
    """
    geolocated_target, _ = compute_geolocation(
        pixel_u=pixel_u,
        pixel_v=pixel_v,
        pixel_confidence=pixel_confidence,
        camera_config=camera_config,
        drone_state=drone_state,
        ground_z=0.0,
        origin_latitude_deg=0.0,
        origin_longitude_deg=0.0,
        origin_altitude_m=0.0,
        range_guard_config=range_guard_config,
        return_debug_info=False,
    )
    return geolocated_target


def compute_geolocation_batch(
    pixels_uv: np.ndarray,
    pixel_confidences: np.ndarray,
    camera_config: CameraConfig,
    drone_state: DroneState,
    ground_z: float = 0.0,
    origin_latitude_deg: float = 0.0,
    origin_longitude_deg: float = 0.0,
    origin_altitude_m: float = 0.0,
    range_guard_config: Optional[RangeGuardConfig] = None,
) -> Tuple[list, list]:
    """
    Geolocate multiple pixels with same drone state.
    
    Args:
        pixels_uv: Array of shape (N, 2) where each row is [u, v]
        pixel_confidences: Array of shape (N,) with confidences
        camera_config: Camera intrinsics and mounting
        drone_state: Drone position and attitude
        ground_z: Ground elevation (metres)
        origin_latitude_deg: Local origin latitude (degrees)
        origin_longitude_deg: Local origin longitude (degrees)
        origin_altitude_m: Local origin altitude (metres)
        range_guard_config: Optional range guard config
    
    Returns:
        (geolocated_targets, errors)
        where geolocated_targets is list of GeolocatedTarget
        and errors is list of (index, exception) for failed pixels
    """
    if pixels_uv.shape[0] != pixel_confidences.shape[0]:
        raise ValueError("pixels_uv and pixel_confidences must have same length")
    
    geolocated_targets = []
    errors = []
    
    for i, ((u, v), conf) in enumerate(zip(pixels_uv, pixel_confidences)):
        try:
            target, _ = compute_geolocation(
                pixel_u=u,
                pixel_v=v,
                pixel_confidence=conf,
                camera_config=camera_config,
                drone_state=drone_state,
                ground_z=ground_z,
                origin_latitude_deg=origin_latitude_deg,
                origin_longitude_deg=origin_longitude_deg,
                origin_altitude_m=origin_altitude_m,
                range_guard_config=range_guard_config,
                return_debug_info=False,
            )
            geolocated_targets.append(target)
        except GeolocationError as e:
            errors.append((i, e))
    
    return (geolocated_targets, errors)
