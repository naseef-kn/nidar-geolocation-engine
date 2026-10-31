"""
Connects the geolocation engine to the drone's reporting.

This module does no geometry and no range validation of its own. It converts
a PixelDetection into coordinates using compute_geolocation, then reports the
result through Drone.report_target. Errors from the engine propagate unchanged.
"""

from nidar.camera_config import CameraConfig
from nidar.detection import PixelDetection
from nidar.drone import Drone
from nidar.geolocated_target import GeolocatedTarget
from nidar.geolocation import DroneState, compute_geolocation


class FrameSizeMismatchError(ValueError):
    """The detection's frame size does not match the camera configuration."""


def geolocate_and_report(
    drone: Drone,
    detection: PixelDetection,
    drone_state: DroneState,
    camera_config: CameraConfig,
    target_id: str,
    phase: str = "outbound",
) -> GeolocatedTarget:
    """
    Geolocate one detection and report it to the GCS.

    Refuses a detection whose frame size differs from the camera config,
    because its pixel coordinates would be in a different scale.

    Returns the GeolocatedTarget so the caller can log it.
    """
    if (
        detection.frame_width != camera_config.image_width
        or detection.frame_height != camera_config.image_height
    ):
        raise FrameSizeMismatchError(
            f"detection frame {detection.frame_width}x{detection.frame_height} "
            f"does not match camera {camera_config.image_width}x"
            f"{camera_config.image_height}"
        )

    target, _debug = compute_geolocation(
        pixel_u=detection.x,
        pixel_v=detection.y,
        pixel_confidence=detection.confidence,
        camera_config=camera_config,
        drone_state=drone_state,
    )

    drone.report_target(
        target_id=target_id,
        latitude=target.latitude_deg,
        longitude=target.longitude_deg,
        confidence=target.confidence,
        phase=phase,
    )
    return target
