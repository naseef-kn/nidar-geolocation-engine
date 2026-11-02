import pytest
import numpy as np

from nidar.camera_config import SYNTHETIC_CAMERA_CONFIG
from nidar.detection import PixelDetection
from nidar.drone import Drone
from nidar.drone_config import DroneConfig
from nidar.geolocated_target import GeolocatedTarget
from nidar.geolocate_report import FrameSizeMismatchError, geolocate_and_report
from nidar.geolocation import DroneState
from nidar.range_guard import RangeGuardAction, RangeGuardConfig
from nidar.transport import GCS_ID, InMemoryTransport

CAM = SYNTHETIC_CAMERA_CONFIG
W, H = CAM.image_width, CAM.image_height


@pytest.fixture
def drone_and_transport():
    transport = InMemoryTransport()
    drone = Drone(DroneConfig(drone_id="alpha", capabilities=("scout",)), transport)
    return drone, transport


def level_state(altitude_m=50.0):
    # NED: z is negative above ground. Level, no rotation.
    return DroneState(
        position_ned_m=(0.0, 0.0, -altitude_m),
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
    )


def centre_detection(confidence=0.9):
    return PixelDetection(
        x=int(CAM.cx),
        y=int(CAM.cy),
        confidence=confidence,
        frame_width=W,
        frame_height=H,
    )


def test_report_sent_to_gcs_with_geolocated_coordinates(drone_and_transport):
    drone, transport = drone_and_transport
    target = geolocate_and_report(
        drone, centre_detection(), level_state(), CAM, target_id="t1", phase="wayback"
    )
    message = transport.receive(GCS_ID)
    assert message.msg_type == "target_found"
    assert message.payload["target_id"] == "t1"
    assert message.payload["latitude"] == target.latitude_deg
    assert message.payload["longitude"] == target.longitude_deg
    assert message.payload["phase"] == "wayback"


def test_reported_confidence_is_engine_confidence(drone_and_transport):
    drone, transport = drone_and_transport
    target = geolocate_and_report(
        drone, centre_detection(0.9), level_state(), CAM, target_id="t1"
    )
    assert transport.receive(GCS_ID).payload["confidence"] == target.confidence


def test_returns_geolocated_target(drone_and_transport):
    drone, _ = drone_and_transport
    target = geolocate_and_report(
        drone, centre_detection(), level_state(), CAM, target_id="t1"
    )
    assert hasattr(target, "latitude_deg")
    assert hasattr(target, "longitude_deg")


def test_frame_size_mismatch_refused_and_nothing_sent(drone_and_transport):
    drone, transport = drone_and_transport
    wrong = PixelDetection(
        x=10,
        y=10,
        confidence=0.9,
        frame_width=W + 1,
        frame_height=H,
    )
    with pytest.raises(FrameSizeMismatchError):
        geolocate_and_report(drone, wrong, level_state(), CAM, target_id="t1")
    assert transport.receive(GCS_ID) is None


def test_default_phase_is_outbound(drone_and_transport):
    drone, transport = drone_and_transport
    geolocate_and_report(drone, centre_detection(), level_state(), CAM, target_id="t1")
    assert transport.receive(GCS_ID).payload["phase"] == "outbound"


def strict_guard(action):
    # 1 degree limit: any off-centre pixel is out of range.
    return RangeGuardConfig(
        max_off_nadir_angle_rad=np.radians(1.0),
        medium_confidence_off_nadir_rad=np.radians(2.0),
        action_on_exceed_max_off_nadir=action,
    )


def edge_detection(confidence=0.9):
    # Far left edge: roughly 28 degrees off nadir at the synthetic focal length.
    return PixelDetection(
        x=0,
        y=int(CAM.cy),
        confidence=confidence,
        frame_width=W,
        frame_height=H,
    )


def test_valid_target_is_reported(drone_and_transport):
    drone, transport = drone_and_transport
    target = geolocate_and_report(
        drone, centre_detection(), level_state(), CAM, target_id="t1"
    )
    assert target.is_valid is True
    assert transport.receive(GCS_ID).msg_type == "target_found"


def test_rejected_target_is_not_reported(drone_and_transport):
    drone, transport = drone_and_transport
    target = geolocate_and_report(
        drone,
        edge_detection(),
        level_state(),
        CAM,
        target_id="t1",
        range_guard_config=strict_guard(RangeGuardAction.REJECT),
    )
    assert target.is_valid is False
    assert transport.receive(GCS_ID) is None


def test_rejected_target_is_still_returned(drone_and_transport):
    drone, _ = drone_and_transport
    target = geolocate_and_report(
        drone,
        edge_detection(),
        level_state(),
        CAM,
        target_id="t1",
        range_guard_config=strict_guard(RangeGuardAction.REJECT),
    )
    assert isinstance(target, GeolocatedTarget)


def test_off_nadir_is_none_when_unknown():
    target = GeolocatedTarget(
        latitude_deg=12.97,
        longitude_deg=77.59,
        altitude_m=0.0,
        confidence=0.5,
        pixel_u=0.0,
        pixel_v=0.0,
        pixel_confidence=0.5,
        slant_range_m=50.0,
        off_nadir_angle_deg=None,
        validation_confidence="low",
    )
    assert target.off_nadir_angle_deg is None
    assert target.is_valid is True


def test_string_representation_handles_unknown_off_nadir():
    target = GeolocatedTarget(
        latitude_deg=12.97,
        longitude_deg=77.59,
        altitude_m=0.0,
        confidence=0.5,
        pixel_u=0.0,
        pixel_v=0.0,
        pixel_confidence=0.5,
        slant_range_m=50.0,
        off_nadir_angle_deg=None,
        validation_confidence="low",
    )
    assert "n/a" in str(target)


def test_flagged_medium_confidence_target_is_reported(drone_and_transport):
    drone, transport = drone_and_transport
    guard = RangeGuardConfig(
        max_off_nadir_angle_rad=np.radians(20.0),
        medium_confidence_off_nadir_rad=np.radians(30.0),
        action_on_exceed_max_off_nadir=RangeGuardAction.FLAG_LOW_CONFIDENCE,
    )

    target = geolocate_and_report(
        drone,
        edge_detection(confidence=0.9),
        level_state(),
        CAM,
        target_id="t1",
        range_guard_config=guard,
    )

    assert target.is_valid is True
    assert target.validation_confidence == "medium"
    assert target.confidence == pytest.approx(0.9 * 0.7)
    message = transport.receive(GCS_ID)
    assert message.payload["confidence"] == pytest.approx(0.9 * 0.7)
