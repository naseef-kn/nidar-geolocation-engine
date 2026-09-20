"""
Test suite for camera_config module.

Tests camera configuration validation, intrinsics, and mounting parameters.
"""

import pytest
import numpy as np
from nidar.camera_config import CameraConfig, SYNTHETIC_CAMERA_CONFIG, validate_camera_config


class TestCameraConfigValidation:
    """Test CameraConfig validation checks."""
    
    def test_synthetic_config_is_valid(self):
        """Synthetic configuration must pass all validation checks."""
        config = SYNTHETIC_CAMERA_CONFIG
        # If this doesn't raise, config is valid
        assert validate_camera_config(config)
    
    def test_config_rejects_zero_image_width(self):
        """Image width must be positive."""
        with pytest.raises(ValueError, match="image_width must be > 0"):
            CameraConfig(
                image_width=0,
                image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_zero_image_height(self):
        """Image height must be positive."""
        with pytest.raises(ValueError, match="image_height must be > 0"):
            CameraConfig(
                image_width=1280,
                image_height=0,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_negative_focal_length_fx(self):
        """Focal length fx must be positive."""
        with pytest.raises(ValueError, match="fx.*must be > 0"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=-1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_negative_focal_length_fy(self):
        """Focal length fy must be positive."""
        with pytest.raises(ValueError, match="fy.*must be > 0"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=-1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_principal_point_out_of_bounds_x(self):
        """Principal point cx must be within [0, image_width-1]."""
        with pytest.raises(ValueError, match="cx.*must be in"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=1280.0, cy=360.0,  # Out of bounds
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_principal_point_out_of_bounds_y(self):
        """Principal point cy must be within [0, image_height-1]."""
        with pytest.raises(ValueError, match="cy.*must be in"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=720.0,  # Out of bounds
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_non_orthogonal_rotation(self):
        """Rotation matrix must be orthonormal."""
        non_orthogonal = np.array([
            [1.0, 0.1, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        with pytest.raises(ValueError, match="not orthonormal"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=non_orthogonal,
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_reflection_matrix(self):
        """Rotation must have det=+1 (not a reflection with det=-1)."""
        reflection = np.diag([1.0, 1.0, -1.0])  # det = -1
        with pytest.raises(ValueError, match="determinant"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=reflection,
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_wrong_rotation_shape(self):
        """Rotation matrix must be 3×3."""
        wrong_shape = np.eye(4)
        with pytest.raises(ValueError, match="3×3"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=wrong_shape,
                offset_cam_in_body=(0, 0, 0.1),
            )
    
    def test_config_rejects_offset_wrong_length(self):
        """Offset must have exactly 3 components."""
        with pytest.raises(ValueError, match="3 components"):
            CameraConfig(
                image_width=1280, image_height=720,
                fx=1200.0, fy=1200.0, cx=640.0, cy=360.0,
                R_cam_to_body=np.eye(3),
                offset_cam_in_body=(0, 0),  # Only 2 components
            )


class TestCameraConfigProperties:
    """Test CameraConfig property methods."""
    
    def test_principal_point_property(self):
        """Principal point property should return (cx, cy) tuple."""
        config = SYNTHETIC_CAMERA_CONFIG
        pp = config.principal_point
        assert pp == (640.0, 360.0)
    
    def test_focal_length_property(self):
        """Focal length property should return (fx, fy) tuple."""
        config = SYNTHETIC_CAMERA_CONFIG
        fl = config.focal_length
        assert fl == (1200.0, 1200.0)
    
    def test_intrinsics_matrix_property(self):
        """Intrinsics matrix should be 3×3 with correct layout."""
        config = SYNTHETIC_CAMERA_CONFIG
        K = config.intrinsics_matrix
        
        assert K.shape == (3, 3)
        assert K[0, 0] == 1200.0  # fx
        assert K[1, 1] == 1200.0  # fy
        assert K[0, 2] == 640.0   # cx
        assert K[1, 2] == 360.0   # cy
        assert K[2, 2] == 1.0
        # Off-diagonal elements should be zero
        assert K[0, 1] == 0.0
        assert K[1, 0] == 0.0


class TestMountingA:
    """Test Mounting A rotation matrix (image-top = drone nose)."""
    
    def test_mounting_a_orthonormal(self):
        """Mounting A rotation must be orthonormal."""
        config = SYNTHETIC_CAMERA_CONFIG
        R = config.R_cam_to_body
        RTR = R @ R.T
        assert np.allclose(RTR, np.eye(3), atol=1e-6)
    
    def test_mounting_a_proper_rotation(self):
        """Mounting A rotation must have det=+1."""
        config = SYNTHETIC_CAMERA_CONFIG
        det = np.linalg.det(config.R_cam_to_body)
        assert np.isclose(det, 1.0, atol=1e-6)
    
    def test_mounting_a_camera_x_maps_to_body_y(self):
        """Camera +X (right) should map to body −Y (port)."""
        config = SYNTHETIC_CAMERA_CONFIG
        R = config.R_cam_to_body
        camera_x = np.array([1, 0, 0])
        body_y_target = np.array([0, 1, 0])
        body_result = R @ camera_x
        assert np.allclose(body_result, body_y_target, atol=1e-6)
    
    def test_mounting_a_camera_y_maps_to_body_x(self):
        """Camera +Y (down in image) should map to body +X (forward/nose)."""
        config = SYNTHETIC_CAMERA_CONFIG
        R = config.R_cam_to_body
        camera_y = np.array([0, 1, 0])
        body_x_target = np.array([-1, 0, 0])
        body_result = R @ camera_y
        assert np.allclose(body_result, body_x_target, atol=1e-6)
    
    def test_mounting_a_camera_z_maps_to_body_z(self):
        """Camera +Z (optical axis, forward) should map to body +Z (down)."""
        config = SYNTHETIC_CAMERA_CONFIG
        R = config.R_cam_to_body
        camera_z = np.array([0, 0, 1])
        body_z_target = np.array([0, 0, 1])
        body_result = R @ camera_z
        assert np.allclose(body_result, body_z_target, atol=1e-6)


class TestSyntheticParameters:
    """Test synthetic Phase 2A configuration values."""
    
    def test_synthetic_image_dimensions(self):
        """Synthetic image should be 1280×720."""
        config = SYNTHETIC_CAMERA_CONFIG
        assert config.image_width == 1280
        assert config.image_height == 720
    
    def test_synthetic_focal_lengths_equal(self):
        """Synthetic focal lengths should be equal (square pixels)."""
        config = SYNTHETIC_CAMERA_CONFIG
        assert config.fx == config.fy
        assert config.fx == 1200.0
    
    def test_synthetic_principal_point_at_centre(self):
        """Synthetic principal point should be at image centre."""
        config = SYNTHETIC_CAMERA_CONFIG
        assert config.cx == 640.0
        assert config.cy == 360.0
    
    def test_synthetic_camera_offset_is_positive_z(self):
        """Camera offset should have +Z component (below body in NED)."""
        config = SYNTHETIC_CAMERA_CONFIG
        assert config.offset_cam_in_body[2] == 0.1
        assert config.offset_cam_in_body[2] > 0


class TestCameraConfigImmutability:
    """Test that CameraConfig is immutable (frozen=True)."""
    
    def test_config_is_frozen(self):
        """CameraConfig should be immutable."""
        config = SYNTHETIC_CAMERA_CONFIG
        with pytest.raises((AttributeError, ValueError)):  # frozen dataclass raises AttributeError
            config.fx = 1000.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
