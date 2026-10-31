import pytest

from nidar.camera_config import SYNTHETIC_CAMERA_CONFIG
from nidar.detection import PixelDetection
from nidar.drone import Drone
from nidar.drone_config import DroneConfig
from nidar.geolocate_report import FrameSizeMismatchError, geolocate_and_report
from nidar.geolocation import DroneState
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
