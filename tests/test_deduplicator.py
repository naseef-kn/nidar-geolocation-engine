import pytest

from nidar.deduplicator import TargetDeduplicator
from nidar.detection import PixelDetection
from nidar.target import Target


def test_single_valid_target_is_returned() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)

    deduplicator = TargetDeduplicator(spatial_threshold_pixels=50.0)
    result = deduplicator.deduplicate([target])

    assert len(result) == 1
    assert result[0].target_id == 1


def test_two_distant_valid_targets_both_kept() -> None:
    detection1 = PixelDetection(
        x=100, y=100, confidence=0.8, frame_width=1280, frame_height=720
    )
    detection2 = PixelDetection(
        x=600, y=600, confidence=0.85, frame_width=1280, frame_height=720
    )
    target1 = Target(target_id=1, detection=detection1, confidence_threshold=0.5)
    target2 = Target(target_id=2, detection=detection2, confidence_threshold=0.5)

    deduplicator = TargetDeduplicator(spatial_threshold_pixels=50.0)
    result = deduplicator.deduplicate([target1, target2])

    assert len(result) == 2
    # Result is sorted by confidence (descending), so target2 (0.85) comes first
    assert result[0].target_id == 2
    assert result[1].target_id == 1


def test_close_targets_are_deduplicated_higher_confidence_kept() -> None:
    detection1 = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    detection2 = PixelDetection(
        x=650, y=370, confidence=0.7, frame_width=1280, frame_height=720
    )
    # Distance = sqrt((650-640)^2 + (370-360)^2) = sqrt(100 + 100) ≈ 14.1 pixels (within 50)
    target1 = Target(target_id=1, detection=detection1, confidence_threshold=0.5)
    target2 = Target(target_id=2, detection=detection2, confidence_threshold=0.5)

    deduplicator = TargetDeduplicator(spatial_threshold_pixels=50.0)
    result = deduplicator.deduplicate([target1, target2])

    # Only the higher-confidence target (0.9) should remain
    assert len(result) == 1
    assert result[0].target_id == 1
    assert result[0].detection.confidence == 0.9