import pytest

from nidar.delivery_drone import DeliveryDrone
from nidar.delivery_task import DeliveryTask
from nidar.detection import PixelDetection
from nidar.target import Target


def test_delivery_drone_completes_assigned_task() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="assigned")

    drone = DeliveryDrone(drone_id=1)
    completed_task = drone.deliver(task)

    assert completed_task.status == "completed"
    assert completed_task.task_id == 101
    assert completed_task.target == target


def test_delivery_drone_rejects_non_assigned_task() -> None:
    detection = PixelDetection(
        x=640, y=360, confidence=0.9, frame_width=1280, frame_height=720
    )
    target = Target(target_id=1, detection=detection, confidence_threshold=0.5)
    task = DeliveryTask(task_id=101, target=target, status="pending")

    drone = DeliveryDrone(drone_id=1)

    with pytest.raises(ValueError):
        drone.deliver(task)


def test_delivery_drone_rejects_negative_id() -> None:
    with pytest.raises(ValueError):
        DeliveryDrone(drone_id=-1)
        