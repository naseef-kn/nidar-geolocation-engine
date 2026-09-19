import pytest

from nidar.delivery_task import DeliveryTask
from nidar.detection import PixelDetection
from nidar.target import Target


def test_delivery_task_starts_as_pending() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target)

    assert task.status == "pending"


def test_delivery_task_transitions_pending_to_assigned() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="pending")

    assigned_task = task.assign()

    assert assigned_task.status == "assigned"
    assert assigned_task.task_id == 101
    assert assigned_task.target == target
    # Original task is unchanged (immutability)
    assert task.status == "pending"


def test_delivery_task_transitions_assigned_to_completed() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="assigned")

    completed_task = task.complete()

    assert completed_task.status == "completed"
    # Original task is unchanged (immutability)
    assert task.status == "assigned"


def test_delivery_task_rejects_invalid_status() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)

    with pytest.raises(ValueError):
        DeliveryTask(task_id=101, target=target, status="unknown")


def test_delivery_task_cannot_assign_non_pending_task() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="assigned")

    with pytest.raises(ValueError):
        task.assign()


def test_delivery_task_cannot_complete_non_assigned_task() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="pending")

    with pytest.raises(ValueError):
        task.complete()