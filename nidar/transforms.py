"""
NIDAR Transforms Module

Geometric transformations for coordinate frame conversions:
- Camera frame (OpenCV)
- Body frame (FRD — Forward-Right-Down, aviation convention)
- World frame (NED — North-East-Down)
- Geographic frame (latitude, longitude, altitude)

All rotations are proper (determinant = +1, right-handed).
All operations preserve unit norms and orthonormality.

Reference frames:
  Camera (OpenCV): X=right, Y=down, Z=forward (optical axis)
  Body (FRD): X=forward (nose), Y=right (starboard), Z=down (belly)
  World (NED): X=north, Y=east, Z=down (below ground)
  Geographic: latitude (deg), longitude (deg), altitude (m WGS84)

Constants:
  Earth radius (spherical approximation): 6,371,000 m
  All angles in radians internally; conversion to/from degrees explicit.
"""

import numpy as np
from typing import Tuple
import math


# ============================================================================
# CONSTANTS
# ============================================================================

R_EARTH = 6_371_000.0  # Earth radius in metres (spherical approximation)
"""Mean Earth radius (metres). Used for NED ↔ geographic conversion."""


# ============================================================================
# ROTATION MATRICES (Euler angles to rotation matrices)
# ============================================================================

def rotation_x(angle_rad: float) -> np.ndarray:
    """
    Rotation matrix around X-axis (roll).
    
    Args:
        angle_rad: Rotation angle in radians (positive = right wing down)
    
    Returns:
        3×3 rotation matrix
    """
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [1.0,    0.0,   0.0],
        [0.0,      c,    -s],
        [0.0,      s,     c],
    ], dtype=np.float64)


def rotation_y(angle_rad: float) -> np.ndarray:
    """
    Rotation matrix around Y-axis (pitch).
    
    Args:
        angle_rad: Rotation angle in radians (positive = nose up)
    
    Returns:
        3×3 rotation matrix
    """
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [   c,   0.0,     s],
        [ 0.0,   1.0,   0.0],
        [  -s,   0.0,     c],
    ], dtype=np.float64)


def rotation_z(angle_rad: float) -> np.ndarray:
    """
    Rotation matrix around Z-axis (yaw/heading).
    
    Args:
        angle_rad: Rotation angle in radians (positive = nose rotates left in top-down view)
    
    Returns:
        3×3 rotation matrix
    
    Notes:
        In NED frame: +Z points down, so +yaw follows right-hand rule about +Z.
        Looking down (from above), +yaw rotates the nose counter-clockwise (north → east).
    """
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([
        [   c,    -s,   0.0],
        [   s,     c,   0.0],
        [ 0.0,   0.0,   1.0],
    ], dtype=np.float64)


def euler_to_rotation(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """
    Convert Euler angles (roll, pitch, yaw) to rotation matrix.
    
    Composition: R = R_z(yaw) @ R_y(pitch) @ R_x(roll)
    
    Order: ZYX (yaw first, then pitch, then roll). This is standard for aviation.
    
    Args:
        roll_rad: Roll angle in radians (+X axis rotation, positive = right wing down)
        pitch_rad: Pitch angle in radians (+Y axis rotation, positive = nose up)
        yaw_rad: Yaw angle in radians (+Z axis rotation, positive = nose toward east)
    
    Returns:
        3×3 rotation matrix (orthonormal, determinant = +1)
    
    Notes:
        This represents the rotation from body frame to world frame (inertial).
        Applied as: v_world = R @ v_body
    """
    R_z = rotation_z(yaw_rad)
    R_y = rotation_y(pitch_rad)
    R_x = rotation_x(roll_rad)
    return R_z @ R_y @ R_x


# ============================================================================
# RIGID BODY TRANSFORMS
# ============================================================================

def apply_rigid_transform(
    position: np.ndarray,
    rotation: np.ndarray,
    origin: np.ndarray,
) -> np.ndarray:
    """
    Apply rigid body transform: p_new = R @ p_old + origin
    
    Args:
        position: Position vector in local frame
        rotation: 3×3 rotation matrix
        origin: Translation origin
    
    Returns:
        Transformed position in global frame
    """
    return rotation @ position + origin


# ============================================================================
# NED ↔ GEOGRAPHIC CONVERSION
# ============================================================================

def ned_to_geographic(
    north_m: float,
    east_m: float,
    down_m: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> Tuple[float, float, float]:
    """
    Convert NED coordinates to geographic (lat, lon, alt) WGS84.
    
    Uses spherical Earth approximation with radius R_EARTH.
    
    Args:
        north_m: Displacement north from origin (metres)
        east_m: Displacement east from origin (metres)
        down_m: Displacement down from origin (metres, positive downward)
        origin_latitude_deg: Origin latitude in degrees
        origin_longitude_deg: Origin longitude in degrees
        origin_altitude_m: Origin altitude in metres (WGS84 ellipsoidal height)
    
    Returns:
        (latitude_deg, longitude_deg, altitude_m)
    
    Notes:
        Spherical approximation assumes Earth is a sphere of radius R_EARTH.
        For small displacements (< 100 km), error is < 0.1 m.
    """
    
    # Convert origin latitude to radians for trig
    lat0_rad = np.radians(origin_latitude_deg)
    
    # Latitude displacement: north displacement / Earth radius
    dlat_rad = north_m / R_EARTH
    lat_rad = lat0_rad + dlat_rad
    lat_deg = np.degrees(lat_rad)
    
    # Longitude displacement: east displacement / (Earth radius * cos(latitude))
    # Use origin latitude for the cos factor (slight approximation)
    cos_lat0 = np.cos(lat0_rad)
    dlon_rad = east_m / (R_EARTH * cos_lat0)
    lon_deg = origin_longitude_deg + np.degrees(dlon_rad)
    
    # Altitude: add down displacement (down_m is positive downward, so subtract it)
    alt_m = origin_altitude_m - down_m
    
    return (lat_deg, lon_deg, alt_m)


def geographic_to_ned(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
    origin_altitude_m: float,
) -> Tuple[float, float, float]:
    """
    Convert geographic coordinates (lat, lon, alt) to NED relative to origin.
    
    Args:
        latitude_deg: Target latitude in degrees
        longitude_deg: Target longitude in degrees
        altitude_m: Target altitude in metres (WGS84 ellipsoidal height)
        origin_latitude_deg: Origin latitude in degrees
        origin_longitude_deg: Origin longitude in degrees
        origin_altitude_m: Origin altitude in metres (WGS84 ellipsoidal height)
    
    Returns:
        (north_m, east_m, down_m) relative to origin
    """
    
    # Convert to radians
    lat_rad = np.radians(latitude_deg)
    lon_rad = np.radians(longitude_deg)
    lat0_rad = np.radians(origin_latitude_deg)
    lon0_rad = np.radians(origin_longitude_deg)
    
    # Latitude difference
    dlat_rad = lat_rad - lat0_rad
    north_m = dlat_rad * R_EARTH
    
    # Longitude difference (must account for cos(latitude))
    dlon_rad = lon_rad - lon0_rad
    cos_lat0 = np.cos(lat0_rad)
    east_m = dlon_rad * R_EARTH * cos_lat0
    
    # Altitude difference (altitude decrease → positive down in NED)
    down_m = origin_altitude_m - altitude_m
    
    return (north_m, east_m, down_m)


# ============================================================================
# UNIT NORM PRESERVATION (validation utilities)
# ============================================================================

def norm_3d(vector: np.ndarray) -> float:
    """Compute Euclidean norm of 3D vector."""
    return float(np.linalg.norm(vector))


def normalize_3d(vector: np.ndarray) -> np.ndarray:
    """Normalize 3D vector to unit length."""
    n = norm_3d(vector)
    if n == 0:
        raise ValueError("Cannot normalize zero vector")
    return vector / n


def is_unit_norm(vector: np.ndarray, atol: float = 1e-6) -> bool:
    """Check if vector has unit norm."""
    return np.isclose(norm_3d(vector), 1.0, atol=atol)


def is_orthonormal(matrix: np.ndarray, atol: float = 1e-6) -> bool:
    """Check if 3×3 matrix is orthonormal (R @ R^T = I)."""
    if matrix.shape != (3, 3):
        return False
    RTR = matrix @ matrix.T
    I = np.eye(3)
    return np.allclose(RTR, I, atol=atol)


def is_proper_rotation(matrix: np.ndarray, atol: float = 1e-6) -> bool:
    """Check if 3×3 matrix is a proper rotation (orthonormal and det=+1)."""
    if not is_orthonormal(matrix, atol):
        return False
    det = float(np.linalg.det(matrix))
    return np.isclose(det, 1.0, atol=atol)


# ============================================================================
# TEST UTILITIES
# ============================================================================

def rotation_matrix_angle_between(
    R1: np.ndarray,
    R2: np.ndarray,
) -> float:
    """
    Compute rotation angle between two rotation matrices.
    
    angle = arccos((trace(R1^T @ R2) - 1) / 2)
    
    Returns angle in radians, [0, π].
    """
    if not (is_proper_rotation(R1) and is_proper_rotation(R2)):
        raise ValueError("Both matrices must be proper rotation matrices")
    
    trace = np.trace(R1.T @ R2)
    # Clamp to [-1, 1] to avoid numerical errors in arccos
    cos_angle = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cos_angle))
