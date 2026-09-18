import pytest

from nidar.detection import PixelDetection


def test_valid_detection_is_created() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    assert detection.x == 640
    assert detection.y == 360
    assert detection.confidence == 0.9


def test_x_outside_frame_is_rejected() -> None:
    with pytest.raises(ValueError):
        PixelDetection(
            x=1280, y=360, confidence=0.9, frame_width=1280, frame_height=720
        )


def test_negative_confidence_is_rejected() -> None:
    with pytest.raises(ValueError):
        PixelDetection(
            x=640, y=360, confidence=-0.1, frame_width=1280, frame_height=720
        )