"""
Test suite for range_guard module.

Tests off-nadir angle validation, confidence levels, and action types.
"""

import pytest
import numpy as np

from nidar.range_guard import (
    validate_geolocation,
    RangeGuardConfig,
    RangeGuardAction,
    ValidationResult,
    SYNTHETIC_RANGE_GUARD_CONFIG,
)


class TestRangeGuardValidation:
    """Test range guard validation logic."""
    
    def test_nadir_ray_passes_guard(self):
        """Nadir ray (straight down) should pass with high confidence."""
        ray_down = np.array([0.0, 0.0, 1.0])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        result = validate_geolocation(
            slant_range_m=50.0,
            ray_world=ray_down,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=SYNTHETIC_RANGE_GUARD_CONFIG,
        )
        
        assert result.is_valid
        assert result.confidence_level == "high"
        assert result.off_nadir_angle_deg == pytest.approx(0.0, abs=1e-3)
    
    def test_30_degree_off_nadir_passes(self):
        """30° off-nadir should pass (well within 60° limit)."""
        angle = np.radians(30.0)
        ray = np.array([np.sin(angle), 0.0, np.cos(angle)])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        result = validate_geolocation(
            slant_range_m=57.7,
            ray_world=ray,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=SYNTHETIC_RANGE_GUARD_CONFIG,
        )
        
        assert result.is_valid
        assert result.confidence_level == "high"
        assert result.off_nadir_angle_deg == pytest.approx(30.0, abs=1e-3)
    
    def test_65_degree_off_nadir_flagged_low_confidence(self):
        """65° off-nadir exceeds 60° max but within 75° medium range → medium confidence."""
        angle = np.radians(65.0)
        ray = np.array([np.sin(angle), 0.0, np.cos(angle)])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        config = RangeGuardConfig(
            enabled=True,
            max_off_nadir_angle_rad=np.radians(60.0),
            medium_confidence_off_nadir_rad=np.radians(75.0),
            action_on_exceed_max_off_nadir=RangeGuardAction.FLAG_LOW_CONFIDENCE,
        )
        
        result = validate_geolocation(
            slant_range_m=129.9,
            ray_world=ray,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=config,
        )
        
        assert result.is_valid
        assert result.confidence_level == "medium"
        assert result.off_nadir_angle_deg == pytest.approx(65.0, abs=1e-3)
    
    def test_80_degree_off_nadir_rejected(self):
        """80° off-nadir exceeds all limits → rejected."""
        angle = np.radians(80.0)
        ray = np.array([np.sin(angle), 0.0, np.cos(angle)])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        config = RangeGuardConfig(
            enabled=True,
            max_off_nadir_angle_rad=np.radians(60.0),
            medium_confidence_off_nadir_rad=np.radians(75.0),
            action_on_exceed_max_off_nadir=RangeGuardAction.REJECT,
        )
        
        result = validate_geolocation(
            slant_range_m=288.2,
            ray_world=ray,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=config,
        )
        
        assert not result.is_valid
        assert result.confidence_level == "low"
    
    def test_guard_disabled_accepts_all(self):
        """With guard disabled, all rays pass."""
        angle = np.radians(85.0)
        ray = np.array([np.sin(angle), 0.0, np.cos(angle)])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        config = RangeGuardConfig(enabled=False)
        
        result = validate_geolocation(
            slant_range_m=572.9,
            ray_world=ray,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=config,
        )
        
        assert result.is_valid
        assert result.confidence_level == "high"
    
    def test_parallel_to_ground_rejected(self):
        """Ray parallel to ground (z ≈ 0) is rejected."""
        ray_horizontal = np.array([1.0, 0.0, 1e-8])  # Nearly parallel
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        result = validate_geolocation(
            slant_range_m=1e10,  # Very large
            ray_world=ray_horizontal,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=SYNTHETIC_RANGE_GUARD_CONFIG,
        )
        
        assert not result.is_valid
    
    def test_negative_slant_range_rejected(self):
        """Negative slant range (ground behind camera) is rejected."""
        ray_down = np.array([0.0, 0.0, 1.0])
        camera_origin = np.array([0.0, 0.0, -50.0])
        
        result = validate_geolocation(
            slant_range_m=-50.0,  # Negative!
            ray_world=ray_down,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=SYNTHETIC_RANGE_GUARD_CONFIG,
        )
        
        assert not result.is_valid
    
    def test_camera_below_ground_rejected(self):
        """Camera at or below ground is rejected."""
        ray_down = np.array([0.0, 0.0, 1.0])
        camera_origin = np.array([0.0, 0.0, 10.0])  # Above ground (z > 0)
        
        result = validate_geolocation(
            slant_range_m=10.0,
            ray_world=ray_down,
            camera_origin_world=camera_origin,
            ground_z=0.0,
            guard_config=SYNTHETIC_RANGE_GUARD_CONFIG,
        )
        
        assert not result.is_valid


class TestRangeGuardConfig:
    """Test RangeGuardConfig validation and properties."""
    
    def test_synthetic_config_is_valid(self):
        """Synthetic range guard config must be valid."""
        config = SYNTHETIC_RANGE_GUARD_CONFIG
        # If __post_init__ didn't raise, config is valid
        assert config.enabled
        assert config.max_off_nadir_angle_rad > 0
    
    def test_config_rejects_invalid_off_nadir_angle(self):
        """Off-nadir angle must be in [0, π/2]."""
        with pytest.raises(ValueError, match="must be in"):
            RangeGuardConfig(
                max_off_nadir_angle_rad=np.pi,  # π/2 is limit, π is too large
            )
    
    def test_config_rejects_medium_less_than_max(self):
        """Medium confidence angle must be >= max off-nadir angle."""
        with pytest.raises(ValueError, match="must be >="):
            RangeGuardConfig(
                max_off_nadir_angle_rad=np.radians(60.0),
                medium_confidence_off_nadir_rad=np.radians(50.0),  # Less than max!
            )


class TestOffNadirAngleComputation:
    """Test off-nadir angle calculation."""
    
    def test_off_nadir_angle_0_degrees(self):
        """Straight down ray has 0° off-nadir."""
        ray = np.array([0.0, 0.0, 1.0])
        angle = np.arccos(abs(ray[2]))
        assert np.isclose(np.degrees(angle), 0.0, atol=1e-6)
    
    def test_off_nadir_angle_45_degrees(self):
        """45° off-nadir ray."""
        angle_rad = np.radians(45.0)
        ray = np.array([np.sin(angle_rad), 0.0, np.cos(angle_rad)])
        computed_angle = np.arccos(abs(ray[2]))
        assert np.isclose(np.degrees(computed_angle), 45.0, atol=1e-6)
    
    def test_off_nadir_angle_60_degrees(self):
        """60° off-nadir ray."""
        angle_rad = np.radians(60.0)
        ray = np.array([np.sin(angle_rad), 0.0, np.cos(angle_rad)])
        computed_angle = np.arccos(abs(ray[2]))
        assert np.isclose(np.degrees(computed_angle), 60.0, atol=1e-6)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
