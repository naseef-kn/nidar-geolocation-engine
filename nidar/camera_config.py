"""
NIDAR Camera Configuration Module

Isolates all camera-specific parameters in a single immutable configuration object.
The geolocation engine accepts CameraConfig as input and never hard-codes camera values.

This enables seamless transition from synthetic Phase 2A → real calibration in Phase 2B
without modifying any geolocation code.

Phase 2A: Synthetic parameters (clearly marked).
Phase 2B+: Replace with real calibration values from OpenCV checkerboard calibration.
"""

from dataclasses import dataclass, field
import numpy as np
from typing import Tuple


@dataclass(frozen=True)
class CameraConfig:
    """
    All camera-specific parameters isolated in one immutable object.
    
    Attributes:
        image_width (int): Image width in pixels
        image_height (int): Image height in pixels
        fx (float): Focal length in pixels (x-direction)
        fy (float): Focal length in pixels (y-direction)
        cx (float): Principal point x-coordinate in pixels
        cy (float): Principal point y-coordinate in pixels
        R_cam_to_body (np.ndarray): 3×3 rotation matrix (camera frame → body frame)
        offset_cam_in_body (tuple): (x, y, z) camera position in body frame (metres)
        distortion_enabled (bool): Whether lens distortion correction is applied
        k1 (float): Radial distortion coefficient (1st order)
        k2 (float): Radial distortion coefficient (2nd order)
        p1 (float): Tangential distortion coefficient (x)
        p2 (float): Tangential distortion coefficient (y)
    
    Notes:
        - This is the ONLY place camera parameters are defined.
        - To use real camera calibration, create a new CameraConfig with measured values.
        - No changes to geolocation engine code are needed.
    """
    
    # IMAGE GEOMETRY
    image_width: int
    image_height: int
    
    # INTRINSICS (pinhole camera model, OpenCV convention)
    fx: float
    fy: float
    cx: float
    cy: float
    
    # EXTRINSICS (camera mounting on drone body)
    R_cam_to_body: np.ndarray
    offset_cam_in_body: Tuple[float, float, float]
    
    # DISTORTION (Phase 2A: disabled; Phase 2B+: real coefficients)
    distortion_enabled: bool = False
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    
    def __post_init__(self) -> None:
        """
        Validate camera configuration after instantiation.
        
        Raises:
            ValueError: If any parameter is invalid
        """
        # IMAGE BOUNDS
        if self.image_width <= 0:
            raise ValueError(
                f"image_width must be > 0, got {self.image_width}"
            )
        if self.image_height <= 0:
            raise ValueError(
                f"image_height must be > 0, got {self.image_height}"
            )
        
        # FOCAL LENGTH (must be positive)
        if self.fx <= 0:
            raise ValueError(
                f"fx (focal length x) must be > 0, got {self.fx}"
            )
        if self.fy <= 0:
            raise ValueError(
                f"fy (focal length y) must be > 0, got {self.fy}"
            )
        
        # PRINCIPAL POINT (must be within image bounds)
        if not (0 <= self.cx <= self.image_width - 1):
            raise ValueError(
                f"cx (principal point x) must be in [0, {self.image_width - 1}], got {self.cx}"
            )
        if not (0 <= self.cy <= self.image_height - 1):
            raise ValueError(
                f"cy (principal point y) must be in [0, {self.image_height - 1}], got {self.cy}"
            )
        
        # ROTATION MATRIX (must be 3×3)
        if self.R_cam_to_body.shape != (3, 3):
            raise ValueError(
                f"R_cam_to_body must be 3×3 array, got shape {self.R_cam_to_body.shape}"
            )
        
        # Check orthonormality: R @ R^T = I
        RTR = self.R_cam_to_body @ self.R_cam_to_body.T
        I = np.eye(3)
        if not np.allclose(RTR, I, atol=1e-6):
            raise ValueError(
                "R_cam_to_body is not orthonormal: R @ R^T ≠ I. "
                "Use np.linalg.qr() or scipy.spatial.transform.Rotation to construct valid rotations."
            )
        
        # Check determinant: must be +1 (proper rotation, not reflection)
        det = float(np.linalg.det(self.R_cam_to_body))
        if not np.isclose(det, 1.0, atol=1e-6):
            raise ValueError(
                f"R_cam_to_body determinant is {det:.6f}, expected +1.0 (proper rotation, not reflection)"
            )
        
        # CAMERA OFFSET (must have exactly 3 components)
        if len(self.offset_cam_in_body) != 3:
            raise ValueError(
                f"offset_cam_in_body must have 3 components (x, y, z), got {len(self.offset_cam_in_body)}"
            )
        
        # Validate offset is numeric
        try:
            offset_array = np.array(self.offset_cam_in_body, dtype=np.float64)
        except (TypeError, ValueError):
            raise ValueError(
                f"offset_cam_in_body components must be numeric, got {self.offset_cam_in_body}"
            )
        
        # DISTORTION COEFFICIENTS
        if self.distortion_enabled:
            for coeff_name, coeff_val in [("k1", self.k1), ("k2", self.k2), ("p1", self.p1), ("p2", self.p2)]:
                if not isinstance(coeff_val, (int, float)):
                    raise ValueError(
                        f"Distortion coefficient {coeff_name} must be numeric, got {type(coeff_val)}"
                    )
    
    @property
    def principal_point(self) -> Tuple[float, float]:
        """Return principal point as (cx, cy) tuple."""
        return (self.cx, self.cy)
    
    @property
    def focal_length(self) -> Tuple[float, float]:
        """Return focal length as (fx, fy) tuple."""
        return (self.fx, self.fy)
    
    @property
    def intrinsics_matrix(self) -> np.ndarray:
        """
        Return camera intrinsics as 3×3 matrix (OpenCV convention).
        
        Returns:
            [[fx,  0, cx],
             [ 0, fy, cy],
             [ 0,  0,  1]]
        """
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)


# ============================================================================
# PHASE 2A SYNTHETIC CAMERA CONFIGURATION
# ============================================================================
# 
# This is SYNTHETIC and for testing/learning only.
# DO NOT USE IN PRODUCTION with real hardware.
#
# Real calibration (Phase 2B+):
# 1. Measure with OpenCV checkerboard calibration
# 2. Create new CameraConfig with measured values
# 3. Pass to geolocation engine
# 4. Zero changes to engine code needed
#
# ============================================================================

SYNTHETIC_CAMERA_CONFIG = CameraConfig(
    # IMAGE: 1280×720 (HD, realistic for RPi Camera Module 3 in 720p mode)
    image_width=1280,
    image_height=720,
    
    # INTRINSICS: Synthetic but physically plausible
    # These are NOT real calibration values from the RPi camera module.
    fx=1200.0,              # SYNTHETIC focal length (pixels)
    fy=1200.0,              # SYNTHETIC focal length (pixels), square aspect
    cx=640.0,               # principal point at image centre
    cy=360.0,               # principal point at image centre
    
    # EXTRINSICS: Mounting A (image-top = drone nose)
    # Camera rotated so:
    #   - image +X (right) → body −Y (port/left)
    #   - image +Y (down) → body +X (forward/nose)
    #   - image +Z (forward) → body +Z (down/belly)
    # 
    # This is the standard aviation convention for a nadir-mounted camera.
    # Orthonormality and determinant verified at __post_init__.
    R_cam_to_body=np.array([
        [ 0.0, -1.0,  0.0],
        [ 1.0,  0.0,  0.0],
        [ 0.0,  0.0,  1.0],
    ], dtype=np.float64),
    
    # Camera position relative to body origin (body frame, metres)
    # NED convention: +Z = down
    # Camera mounted 10 cm below body (nadir mounting, typical position)
    offset_cam_in_body=(0.0, 0.0, +0.1),
    
    # DISTORTION: Disabled in Phase 2A
    # Will be populated in Phase 2B with real calibration coefficients
    distortion_enabled=False,
)


def validate_camera_config(config: CameraConfig) -> bool:
    """
    Validate a camera configuration (convenience wrapper).
    
    Args:
        config: CameraConfig to validate
    
    Returns:
        True if valid (__post_init__ already checked, so always returns True)
    
    Raises:
        ValueError: if config is invalid
    """
    # __post_init__ already ran; if we got here, config is valid
    return True
