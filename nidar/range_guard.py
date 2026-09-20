"""
NIDAR Range/Validity Guard Module

Modular validation layer that applies geometric sanity checks to geolocation results.
Separates guard logic from core geolocation engine.

The guard checks:
1. Mandatory geometric invariants (non-negotiable)
   - λ > 0 (ground is in front of camera)
   - |ray.z| > ε (ray not parallel to ground)
   - camera above ground (physical validity)

2. Provisional off-nadir angle guard (configurable, MVP default 60°)
   - Altitude-independent (works at any altitude)
   - Physically grounded (foreshortening/occlusion risk)
   - Adjustable post-flight based on accuracy data

Phase 2A: Synthetic config with 60° threshold, FLAG_LOW_CONFIDENCE action.
Phase 2B+: Adjust threshold based on real flight geolocation accuracy data.
"""

from dataclasses import dataclass
from enum import Enum
import numpy as np
from typing import Optional


class RangeGuardAction(Enum):
    """Action to take when a detection violates the off-nadir angle limit."""
    
    REJECT = "reject"
    """Discard entirely (safest, but may miss valid targets)."""
    
    FLAG_LOW_CONFIDENCE = "flag_low"
    """Accept but mark low-confidence (more forgiving for real missions)."""
    
    WARN = "warn"
    """Accept but log warning (for debugging)."""


@dataclass
class ValidationResult:
    """Result of range/validity guard validation."""
    
    is_valid: bool
    """True if detection passes mandatory checks and provisional guard."""
    
    confidence_level: str
    """'high', 'medium', or 'low' — indicates geolocation confidence."""
    
    reason: str
    """Human-readable explanation of validation result."""
    
    off_nadir_angle_deg: Optional[float]
    """Computed off-nadir angle in degrees, or None if not computed."""
    
    slant_range_m: Optional[float]
    """Slant range in metres, or None if not computed."""


@dataclass
class RangeGuardConfig:
    """
    Configuration for range/validity guard (Phase 2A/2B).
    
    This controls only provisional geometric sanity checks.
    The core geolocation engine is unaffected by these parameters.
    
    Attributes:
        enabled (bool): Whether the off-nadir angle guard is active
        max_off_nadir_angle_rad (float): Maximum acceptable off-nadir angle (radians)
        medium_confidence_off_nadir_rad (float): Threshold for "medium" confidence (radians)
        action_on_exceed_max_off_nadir (RangeGuardAction): Action when limit exceeded
    
    Notes:
        Phase 2A MVP defaults:
        - max_off_nadir_angle_rad = 60° = 1.047 rad
        - medium_confidence_off_nadir_rad = 75° = 1.309 rad
        - action = FLAG_LOW_CONFIDENCE (accept but mark low-confidence)
        
        After real flight testing, adjust based on geolocation accuracy vs. off-nadir angle.
    """
    
    enabled: bool = True
    """Whether to apply the off-nadir angle guard."""
    
    max_off_nadir_angle_rad: float = np.radians(60.0)
    """
    Maximum acceptable off-nadir angle (radians).
    
    Phase 2A MVP: 60° (1.047 rad). Conservative, allows corner pixels (~29° off-nadir).
    Post-flight adjustment: may change to 45°, 70°, 80°, or remove entirely based on data.
    """
    
    medium_confidence_off_nadir_rad: float = np.radians(75.0)
    """
    Off-nadir angle threshold for 'medium' confidence (radians).
    
    If max_off_nadir < angle < medium_confidence and action=FLAG_LOW_CONFIDENCE,
    the detection is accepted but marked 'medium' confidence instead of rejected.
    """
    
    action_on_exceed_max_off_nadir: RangeGuardAction = RangeGuardAction.FLAG_LOW_CONFIDENCE
    """
    What to do when off-nadir angle exceeds max_off_nadir_angle_rad.
    
    - REJECT: discard (safest but may miss real targets)
    - FLAG_LOW_CONFIDENCE: accept but mark low-confidence (recommended for MVP)
    - WARN: accept and log warning (debugging)
    """
    
    def __post_init__(self) -> None:
        """Validate guard configuration."""
        # Off-nadir angle must be in [0, π/2] radians
        if not (0 <= self.max_off_nadir_angle_rad <= np.pi / 2):
            raise ValueError(
                f"max_off_nadir_angle_rad must be in [0, π/2], got {self.max_off_nadir_angle_rad:.4f}"
            )
        
        # Medium confidence must be >= max off-nadir
        if self.medium_confidence_off_nadir_rad < self.max_off_nadir_angle_rad:
            raise ValueError(
                f"medium_confidence_off_nadir_rad ({self.medium_confidence_off_nadir_rad:.4f}) "
                f"must be >= max_off_nadir_angle_rad ({self.max_off_nadir_angle_rad:.4f})"
            )


# ============================================================================
# PHASE 2A SYNTHETIC RANGE GUARD CONFIGURATION
# ============================================================================

SYNTHETIC_RANGE_GUARD_CONFIG = RangeGuardConfig(
    enabled=True,
    
    # MVP default: 60° off-nadir (conservative, allows typical edge pixels)
    # After 1st real flight, adjust based on geolocation accuracy data
    max_off_nadir_angle_rad=np.radians(60.0),
    
    # "Medium" confidence threshold: 75° off-nadir
    # Between 60° and 75°, detection is accepted but marked medium-confidence
    medium_confidence_off_nadir_rad=np.radians(75.0),
    
    # Action: FLAG_LOW_CONFIDENCE (safer than REJECT for real missions)
    # Geolocation will try to recover if needed; deduplicator respects confidence levels
    action_on_exceed_max_off_nadir=RangeGuardAction.FLAG_LOW_CONFIDENCE,
)


# ============================================================================
# VALIDATION FUNCTION
# ============================================================================

def validate_geolocation(
    slant_range_m: float,
    ray_world: np.ndarray,
    camera_origin_world: np.ndarray,
    ground_z: float,
    guard_config: RangeGuardConfig,
) -> ValidationResult:
    """
    Apply range/validity guard to a geolocation result.
    
    Args:
        slant_range_m: Distance along ray to ground intersection (metres)
        ray_world: Unit ray direction in world frame (NED), from geolocation
        camera_origin_world: Camera position in world frame (NED), from geolocation
        ground_z: Ground elevation (z-coordinate in NED, typically 0)
        guard_config: RangeGuardConfig with thresholds and action
    
    Returns:
        ValidationResult with is_valid, confidence_level, reason, off_nadir_angle_deg
    
    Notes:
        Mandatory checks (always enforced, regardless of guard_config.enabled):
        - λ > 0 (ground in front of camera)
        - |ray.z| > ε (ray not parallel to ground)
        - camera above ground (physical impossibility check)
        
        Provisional check (only if guard_config.enabled):
        - off_nadir_angle <= max_off_nadir_angle_rad (configurable threshold)
    """
    
    # ========================================================================
    # MANDATORY GEOMETRIC CHECKS (non-negotiable)
    # ========================================================================
    
    # Check 1: Slant range must be positive (ground in front of camera)
    if slant_range_m <= 0:
        return ValidationResult(
            is_valid=False,
            confidence_level="low",
            reason=f"Slant range is non-positive ({slant_range_m:.3f} m) — ground is behind camera (geometry error)",
            off_nadir_angle_deg=None,
            slant_range_m=slant_range_m,
        )
    
    # Check 2: Ray must have non-zero z-component (not parallel to ground)
    # Using small epsilon to avoid numerical issues
    eps = 1e-6
    if abs(ray_world[2]) < eps:
        return ValidationResult(
            is_valid=False,
            confidence_level="low",
            reason=f"Ray z-component is nearly zero ({ray_world[2]:.6f}) — ray parallel to ground (no intersection)",
            off_nadir_angle_deg=None,
            slant_range_m=slant_range_m,
        )
    
    # Check 3: Camera must be above ground (z < 0 in NED, since +Z = down)
    if camera_origin_world[2] >= ground_z:
        return ValidationResult(
            is_valid=False,
            confidence_level="low",
            reason=f"Camera z-position ({camera_origin_world[2]:.3f}) >= ground z ({ground_z:.3f}) — impossible (geometry error)",
            off_nadir_angle_deg=None,
            slant_range_m=slant_range_m,
        )
    
    # ========================================================================
    # PROVISIONAL CHECK: Off-nadir angle guard
    # ========================================================================
    
    # Compute off-nadir angle
    # For unit ray, off_nadir = arccos(|z|)
    # ray[2] is z-component in world frame; positive z means downward (toward ground)
    off_nadir_angle_rad = np.arccos(abs(ray_world[2]))
    off_nadir_angle_deg = np.degrees(off_nadir_angle_rad)
    
    # If guard is disabled, accept everything that passed mandatory checks
    if not guard_config.enabled:
        return ValidationResult(
            is_valid=True,
            confidence_level="high",
            reason="Passes mandatory checks; off-nadir guard disabled",
            off_nadir_angle_deg=off_nadir_angle_deg,
            slant_range_m=slant_range_m,
        )
    
    # Guard is enabled; check off-nadir angle
    
    if off_nadir_angle_rad <= guard_config.max_off_nadir_angle_rad:
        # Within limit: high confidence
        return ValidationResult(
            is_valid=True,
            confidence_level="high",
            reason=f"Off-nadir angle {off_nadir_angle_deg:.1f}° is within limit {np.degrees(guard_config.max_off_nadir_angle_rad):.1f}°",
            off_nadir_angle_deg=off_nadir_angle_deg,
            slant_range_m=slant_range_m,
        )
    
    # Exceeds max but check if within medium confidence range
    if off_nadir_angle_rad <= guard_config.medium_confidence_off_nadir_rad:
        # Between max and medium threshold
        action = guard_config.action_on_exceed_max_off_nadir
        
        if action == RangeGuardAction.FLAG_LOW_CONFIDENCE:
            return ValidationResult(
                is_valid=True,
                confidence_level="medium",
                reason=(
                    f"Off-nadir angle {off_nadir_angle_deg:.1f}° exceeds max {np.degrees(guard_config.max_off_nadir_angle_rad):.1f}° "
                    f"but within medium-confidence range {np.degrees(guard_config.medium_confidence_off_nadir_rad):.1f}°"
                ),
                off_nadir_angle_deg=off_nadir_angle_deg,
                slant_range_m=slant_range_m,
            )
        elif action == RangeGuardAction.WARN:
            return ValidationResult(
                is_valid=True,
                confidence_level="low",
                reason=(
                    f"Warning: off-nadir angle {off_nadir_angle_deg:.1f}° exceeds max {np.degrees(guard_config.max_off_nadir_angle_rad):.1f}° "
                    f"(accepting but low-confidence)"
                ),
                off_nadir_angle_deg=off_nadir_angle_deg,
                slant_range_m=slant_range_m,
            )
    
    # Well beyond limits
    action = guard_config.action_on_exceed_max_off_nadir
    
    if action == RangeGuardAction.REJECT:
        return ValidationResult(
            is_valid=False,
            confidence_level="low",
            reason=(
                f"Off-nadir angle {off_nadir_angle_deg:.1f}° exceeds maximum {np.degrees(guard_config.max_off_nadir_angle_rad):.1f}° "
                f"(rejected)"
            ),
            off_nadir_angle_deg=off_nadir_angle_deg,
            slant_range_m=slant_range_m,
        )
    else:
        # FLAG_LOW_CONFIDENCE or WARN
        return ValidationResult(
            is_valid=True,
            confidence_level="low",
            reason=(
                f"Off-nadir angle {off_nadir_angle_deg:.1f}° exceeds maximum {np.degrees(guard_config.max_off_nadir_angle_rad):.1f}° "
                f"(low-confidence)"
            ),
            off_nadir_angle_deg=off_nadir_angle_deg,
            slant_range_m=slant_range_m,
        )
