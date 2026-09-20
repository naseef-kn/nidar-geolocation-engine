"""
NIDAR Geolocated Target Module

Output data model for the geolocation pipeline.

A GeolocatedTarget wraps:
- Original pixel detection (u, v, confidence from scout drone)
- Computed geographic coordinates (latitude, longitude, altitude)
- Geolocation metadata (slant range, ray direction, off-nadir angle, validation confidence)

GeolocatedTarget flows into Phase 1's Target class (which expects lat/lon/alt + confidence).
The confidence is a product of:
  - Detection confidence (from PixelDetection, 0–1 range)
  - Validation confidence level ("high", "medium", "low")
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class GeolocatedTarget:
    """
    Complete geolocation result with coordinates and metadata.
    
    This is the output of the geolocation pipeline and the input to Phase 1's
    Target class.
    
    Attributes:
        latitude_deg: Target latitude in degrees (WGS84)
        longitude_deg: Target longitude in degrees (WGS84)
        altitude_m: Target altitude in metres (WGS84 ellipsoidal height)
        confidence: Detection confidence (0–1 range, product of pixel confidence and validation)
        pixel_u: Original pixel u-coordinate (0 ≤ u < image_width)
        pixel_v: Original pixel v-coordinate (0 ≤ v < image_height)
        pixel_confidence: Confidence from original PixelDetection (0–1 range)
        slant_range_m: Distance from camera to target along ray (metres)
        off_nadir_angle_deg: Angle from nadir (degrees, 0 = straight down)
        validation_confidence: Geolocation confidence level ("high", "medium", "low")
        ray_world: Unit ray in world frame (NED), for debugging/analysis
        camera_origin_world: Camera position in world frame (NED), for debugging/analysis
    """
    
    # GEOGRAPHIC COORDINATES (primary output)
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    
    # CONFIDENCE (combined from detection + validation)
    confidence: float
    """
    Combined confidence (0–1 range).
    Product of pixel_confidence × validation_confidence_multiplier.
    
    High: confidence × 1.0
    Medium: confidence × 0.7
    Low: confidence × 0.5
    """
    
    # PIXEL ORIGIN (traceability)
    pixel_u: float
    pixel_v: float
    pixel_confidence: float
    """Original detection confidence from scout drone (0–1 range)."""
    
    # GEOLOCATION METADATA
    slant_range_m: float
    """Distance along ray from camera to ground intersection (metres)."""
    
    off_nadir_angle_deg: float
    """Angle from nadir (0 = straight down, 90 = horizontal)."""
    
    validation_confidence: str
    """'high', 'medium', or 'low' — geolocation confidence level."""
    
    # DEBUGGING / ANALYSIS
    ray_world: Optional[np.ndarray] = None
    """Unit ray in world frame (NED), useful for post-processing analysis."""
    
    camera_origin_world: Optional[np.ndarray] = None
    """Camera position in world frame (NED), useful for analysis."""
    
    def __post_init__(self) -> None:
        """Validate geolocated target."""
        # Confidence must be [0, 1]
        if not (0 <= self.confidence <= 1.0):
            raise ValueError(f"confidence must be in [0, 1], got {self.confidence}")
        
        if not (0 <= self.pixel_confidence <= 1.0):
            raise ValueError(f"pixel_confidence must be in [0, 1], got {self.pixel_confidence}")
        
        # Off-nadir angle must be [0, 90]
        if not (0 <= self.off_nadir_angle_deg <= 90.0):
            raise ValueError(f"off_nadir_angle_deg must be in [0, 90], got {self.off_nadir_angle_deg}")
        
        # Slant range must be positive
        if self.slant_range_m <= 0:
            raise ValueError(f"slant_range_m must be > 0, got {self.slant_range_m}")
        
        # Validation confidence must be one of the valid levels
        if self.validation_confidence not in ("high", "medium", "low"):
            raise ValueError(
                f"validation_confidence must be 'high', 'medium', or 'low', got {self.validation_confidence}"
            )
        
        # Ray and camera_origin should both be present or both absent
        if (self.ray_world is None) != (self.camera_origin_world is None):
            raise ValueError(
                "ray_world and camera_origin_world must both be present or both absent"
            )
        
        # If present, validate ray
        if self.ray_world is not None:
            if self.ray_world.shape != (3,):
                raise ValueError(f"ray_world must be shape (3,), got {self.ray_world.shape}")
            ray_norm = float(np.linalg.norm(self.ray_world))
            if not np.isclose(ray_norm, 1.0, atol=1e-6):
                raise ValueError(f"ray_world must be unit norm, got norm {ray_norm}")
        
        # If present, validate camera origin
        if self.camera_origin_world is not None:
            if self.camera_origin_world.shape != (3,):
                raise ValueError(f"camera_origin_world must be shape (3,), got {self.camera_origin_world.shape}")
    
    @staticmethod
    def confidence_multiplier(validation_confidence: str) -> float:
        """
        Return confidence multiplier based on validation level.
        
        Args:
            validation_confidence: 'high', 'medium', or 'low'
        
        Returns:
            Multiplier: high=1.0, medium=0.7, low=0.5
        """
        multipliers = {
            "high": 1.0,
            "medium": 0.7,
            "low": 0.5,
        }
        if validation_confidence not in multipliers:
            raise ValueError(f"Unknown validation_confidence: {validation_confidence}")
        return multipliers[validation_confidence]
    
    def as_phase1_input(self) -> dict:
        """
        Convert to Phase 1 Target input dictionary.
        
        Returns:
            Dictionary with keys: latitude, longitude, altitude, confidence
        
        Notes:
            This is used to create Target(lat, lon, alt, confidence) in Phase 1.
        """
        return {
            "latitude": self.latitude_deg,
            "longitude": self.longitude_deg,
            "altitude": self.altitude_m,
            "confidence": self.confidence,
        }
    
    def __repr__(self) -> str:
        """Human-readable representation."""
        return (
            f"GeolocatedTarget("
            f"lat={self.latitude_deg:.6f}°, lon={self.longitude_deg:.6f}°, alt={self.altitude_m:.1f}m, "
            f"conf={self.confidence:.2f}, off_nadir={self.off_nadir_angle_deg:.1f}°, "
            f"val={self.validation_confidence})"
        )


@dataclass
class GeolocationDebugInfo:
    """
    Debug information from geolocation computation.
    
    Useful for development, testing, and post-flight analysis.
    """
    
    # Input
    pixel_u: float
    pixel_v: float
    drone_position_ned_m: tuple  # (x, y, z)
    drone_attitude_rad: tuple  # (roll, pitch, yaw)
    
    # Intermediate computations
    ray_camera_frame: np.ndarray
    ray_body_frame: np.ndarray
    ray_world_frame: np.ndarray
    camera_origin_world: np.ndarray
    
    # Ground intersection
    slant_range_m: float
    ground_position_ned_m: tuple  # (north, east, down)
    
    # Geographic conversion
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    
    # Validation
    off_nadir_angle_deg: float
    validation_result_reason: str


def combine_confidence_levels(
    pixel_confidence: float,
    validation_confidence: str,
) -> float:
    """
    Combine pixel detection confidence with validation confidence.
    
    Args:
        pixel_confidence: Confidence from PixelDetection (0–1)
        validation_confidence: 'high', 'medium', or 'low'
    
    Returns:
        Combined confidence (0–1)
    
    Formula:
        combined = pixel_confidence × multiplier(validation_confidence)
    
    Example:
        pixel_conf = 0.8, validation = "medium"
        → combined = 0.8 × 0.7 = 0.56
    """
    multiplier = GeolocatedTarget.confidence_multiplier(validation_confidence)
    combined = pixel_confidence * multiplier
    return float(np.clip(combined, 0.0, 1.0))  # Ensure [0, 1] range
