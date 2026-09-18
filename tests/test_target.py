import pytest

from nidar.detection import PixelDetection
from nidar.target import Target


def test_target_with_high_confidence_is_valid() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)

    assert target.is_valid() is True


def test_target_with_low_confidence_is_invalid() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.3, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)

    assert target.is_valid() is False


def test_target_rejects_negative_id() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    with pytest.raises(ValueError):
        Target(target_id=-1, detection=detection, confidence_threshold=0.5)