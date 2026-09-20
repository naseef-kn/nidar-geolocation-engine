"""
NIDAR Geolocation Engine Package

Phase 1: Survivor detection, pixel tracking, deduplication, delivery
Phase 2A: Geolocation (pixel → geographic coordinates with validation)
"""

# ============================================================================
# PHASE 1 IMPORTS (Survivor Detection → Delivery)
# ============================================================================

from .detection import PixelDetection
from .scout_drone import ScoutDrone
from .target import Target
from .deduplicator import TargetDeduplicator
from .delivery_task import DeliveryTask
from .delivery_drone import DeliveryDrone

# ============================================================================
# PHASE 2A IMPORTS (Geolocation Engine)
# ============================================================================

# Camera Configuration (isolated, zero hard-coded parameters)
from .camera_config import CameraConfig, SYNTHETIC_CAMERA_CONFIG

# Range Guard (altitude-independent validation, modular)
from .range_guard import (
    RangeGuardConfig,
    RangeGuardAction,
    validate_geolocation,
    SYNTHETIC_RANGE_GUARD_CONFIG,
)

# Transforms (rotation matrices, coordinate frame conversions)
from .transforms import (
    rotation_x,
    rotation_y,
    rotation_z,
    euler_to_rotation,
    ned_to_geographic,
    geographic_to_ned,
)

# Camera Geometry (pixel → ray conversion)
from .camera_geometry import pixel_to_camera_ray

# Geolocated Target (output data model)
from .geolocated_target import (
    GeolocatedTarget,
    GeolocationDebugInfo,
    combine_confidence_levels,
)

# Geolocation Engine (complete pipeline)
from .geolocation import (
    compute_geolocation,
    compute_geolocation_simple,
    compute_geolocation_batch,
    DroneState,
    GeolocationError,
    TelemetryError,
    NoIntersectionError,
)

# ============================================================================
# PUBLIC API
# ============================================================================

__all__ = [
    # Phase 1
    "PixelDetection",
    "ScoutDrone",
    "Target",
    "TargetDeduplicator",
    "DeliveryTask",
    "DeliveryDrone",
    # Phase 2A
    "CameraConfig",
    "SYNTHETIC_CAMERA_CONFIG",
    "RangeGuardConfig",
    "RangeGuardAction",
    "validate_geolocation",
    "SYNTHETIC_RANGE_GUARD_CONFIG",
    "rotation_x",
    "rotation_y",
    "rotation_z",
    "euler_to_rotation",
    "ned_to_geographic",
    "geographic_to_ned",
    "pixel_to_camera_ray",
    "GeolocatedTarget",
    "GeolocationDebugInfo",
    "combine_confidence_levels",
    "compute_geolocation",
    "compute_geolocation_simple",
    "compute_geolocation_batch",
    "DroneState",
    "GeolocationError",
    "TelemetryError",
    "NoIntersectionError",
]