import pytest

from nidar.scout_drone import ScoutDrone


def test_scout_drone_detects_survivor() -> None:
    scout = ScoutDrone(frame_width=1280, frame_height=720)
    detections = scout.detect_survivors()

    assert len(detections) == 1
    detection = detections[0]
    assert detection.x == 640  # centre_x = 1280 // 2
    assert detection.y == 360  # centre_y = 720 // 2
    assert detection.confidence == 0.95


def test_scout_drone_rejects_invalid_frame_width() -> None:
    with pytest.raises(ValueError):
        ScoutDrone(frame_width=-100, frame_height=720)


def test_scout_drone_rejects_zero_frame_height() -> None:
    with pytest.raises(ValueError):
        ScoutDrone(frame_width=1280, frame_height=0)