"""
NIDAR Camera Geometry Module

Converts pixel coordinates to camera rays using intrinsic calibration.

This module handles the first step of geolocation:
  pixel (u, v) → camera ray (x_c, y_c, z_c) in camera frame

The pinhole camera model:
  - No lens distortion (Phase 2A)
  - Intrinsics: fx, fy, cx, cy
  - Image convention: origin at top-left, u→right, v→down (OpenCV/PIL)
  - Camera convention: Z=forward (optical axis), X=right, Y=down

Output: Unit-norm rays in camera frame (‖ray_cam‖ = 1).
"""

import numpy as np
from typing import Tuple
from .camera_config import CameraConfig


def pixel_to_camera_ray(
    u: float,
    v: float,
    config: CameraConfig,
) -> np.ndarray:
    """
    Convert pixel coordinates to unit ray in camera frame.
    
    Pinhole camera model (no distortion):
      x_c = (u - cx) / fx
      y_c = (v - cy) / fy
      z_c = 1
      ray_cam = [x_c, y_c, z_c] / ‖[x_c, y_c, z_c]‖
    
    Args:
        u: Pixel x-coordinate (0 ≤ u < image_width)
        v: Pixel y-coordinate (0 ≤ v < image_height)
        config: CameraConfig with intrinsics (fx, fy, cx, cy)
    
    Returns:
        Unit-norm ray in camera frame (‖ray‖ = 1)
    
    Raises:
        ValueError: If pixel coordinates are out of bounds
    
    Notes:
        Output ray points in +Z direction (forward along optical axis) with Z > 0.
        Invariant: ‖ray_cam‖ = 1.0
    
    Examples:
        Centre pixel (cx, cy):
            u = 640, v = 360, fx=1200, fy=1200, cx=640, cy=360
            → x_c = 0, y_c = 0, z_c = 1
            → ray = [0, 0, 1] (straight forward)
        
        Right pixel (u > cx):
            u = 880, v = 360, fx=1200, fy=1200, cx=640, cy=360
            → x_c = (880-640)/1200 = 0.2, y_c = 0, z_c = 1
            → ray_unnormalized = [0.2, 0, 1], norm = 1.00995
            → ray = [0.19801, 0, 0.98020]
    """
    
    # Validate pixel bounds
    if not (0 <= u < config.image_width):
        raise ValueError(
            f"Pixel u={u} out of bounds [0, {config.image_width})"
        )
    if not (0 <= v < config.image_height):
        raise ValueError(
            f"Pixel v={v} out of bounds [0, {config.image_height})"
        )
    
    # Pinhole model: backproject to 3D in camera frame
    x_c = (u - config.cx) / config.fx
    y_c = (v - config.cy) / config.fy
    z_c = 1.0
    
    # Normalize to unit ray
    ray_unnormalized = np.array([x_c, y_c, z_c], dtype=np.float64)
    norm = float(np.linalg.norm(ray_unnormalized))
    
    if norm == 0:
        raise ValueError(f"Ray has zero norm (should never happen). ray={ray_unnormalized}")
    
    ray_cam = ray_unnormalized / norm
    
    # Verify invariant
    ray_norm = float(np.linalg.norm(ray_cam))
    if not np.isclose(ray_norm, 1.0, atol=1e-6):
        raise RuntimeError(f"Ray norm is {ray_norm}, expected 1.0 (invariant violation)")
    
    # Verify Z > 0 (ray points forward)
    if ray_cam[2] <= 0:
        raise RuntimeError(f"Ray z-component is {ray_cam[2]}, expected > 0")
    
    return ray_cam


def pixel_to_camera_ray_batch(
    pixels_uv: np.ndarray,
    config: CameraConfig,
) -> np.ndarray:
    """
    Convert batch of pixels to rays in camera frame.
    
    Args:
        pixels_uv: Array of shape (N, 2) where each row is [u, v]
        config: CameraConfig
    
    Returns:
        Array of shape (N, 3) where each row is a unit ray in camera frame
    """
    if pixels_uv.ndim != 2 or pixels_uv.shape[1] != 2:
        raise ValueError(f"pixels_uv must have shape (N, 2), got {pixels_uv.shape}")
    
    rays = np.zeros((pixels_uv.shape[0], 3), dtype=np.float64)
    for i, (u, v) in enumerate(pixels_uv):
        rays[i] = pixel_to_camera_ray(u, v, config)
    
    return rays


def image_bounds_check(
    u: float,
    v: float,
    config: CameraConfig,
) -> Tuple[bool, str]:
    """
    Check if pixel is within image bounds.
    
    Args:
        u: Pixel x-coordinate
        v: Pixel y-coordinate
        config: CameraConfig
    
    Returns:
        (is_valid, reason) tuple
    """
    if not (0 <= u < config.image_width):
        return (False, f"u={u} out of bounds [0, {config.image_width})")
    if not (0 <= v < config.image_height):
        return (False, f"v={v} out of bounds [0, {config.image_height})")
    return (True, "")


def principal_point_offset(
    u: float,
    v: float,
    config: CameraConfig,
) -> Tuple[float, float]:
    """
    Compute offset from principal point in normalised image coordinates.
    
    Args:
        u: Pixel x-coordinate
        v: Pixel y-coordinate
        config: CameraConfig
    
    Returns:
        (x_normalised, y_normalised) offset in normalised image space
    """
    x_norm = (u - config.cx) / config.fx
    y_norm = (v - config.cy) / config.fy
    return (x_norm, y_norm)


def field_of_view(
    config: CameraConfig,
) -> Tuple[float, float]:
    """
    Compute horizontal and vertical field of view.
    
    Args:
        config: CameraConfig
    
    Returns:
        (fov_h_rad, fov_v_rad) field of view in radians
    
    Notes:
        FOV is computed as the angle subtended by the full image width/height.
        For a pinhole camera with symmetric principal point at image centre:
        
        FOV_h = 2 * arctan(width / (2 * fx))
        FOV_v = 2 * arctan(height / (2 * fy))
    """
    # Horizontal FOV: from left edge to right edge
    fov_h = 2.0 * np.arctan(config.image_width / (2.0 * config.fx))
    
    # Vertical FOV: from top edge to bottom edge
    fov_v = 2.0 * np.arctan(config.image_height / (2.0 * config.fy))
    
    return (fov_h, fov_v)


def diagonal_field_of_view(
    config: CameraConfig,
) -> float:
    """
    Compute diagonal field of view (corner pixel to corner pixel through centre).
    
    Returns:
        Diagonal FOV in radians
    """
    # Ray to corner pixel (width-1, height-1)
    ray_corner = pixel_to_camera_ray(
        config.image_width - 1,
        config.image_height - 1,
        config,
    )
    
    # Ray to opposite corner (0, 0)
    ray_opposite = pixel_to_camera_ray(0, 0, config)
    
    # Angle between rays
    cos_angle = float(np.dot(ray_corner, ray_opposite))
    cos_angle = np.clip(cos_angle, -1.0, 1.0)  # Handle numerical errors
    angle = np.arccos(cos_angle)
    
    return angle


# ============================================================================
# PHASE 2A CAMERA GEOMETRY UTILITIES
# ============================================================================

def synthetic_geometry_info(config: CameraConfig) -> dict:
    """
    Print camera geometry info for debugging/documentation.
    
    Args:
        config: CameraConfig
    
    Returns:
        Dictionary with geometry properties
    """
    fov_h, fov_v = field_of_view(config)
    fov_d = diagonal_field_of_view(config)
    
    return {
        "image_width": config.image_width,
        "image_height": config.image_height,
        "fx": config.fx,
        "fy": config.fy,
        "cx": config.cx,
        "cy": config.cy,
        "principal_point": config.principal_point,
        "focal_length": config.focal_length,
        "fov_horizontal_rad": fov_h,
        "fov_horizontal_deg": np.degrees(fov_h),
        "fov_vertical_rad": fov_v,
        "fov_vertical_deg": np.degrees(fov_v),
        "fov_diagonal_rad": fov_d,
        "fov_diagonal_deg": np.degrees(fov_d),
        "aspect_ratio": config.image_width / config.image_height,
    }
