"""End-to-end MVP integration test: scout → validate → deduplicate → deliver → complete."""

from nidar.deduplicator import TargetDeduplicator
from nidar.delivery_drone import DeliveryDrone
from nidar.delivery_task import DeliveryTask
from nidar.scout_drone import ScoutDrone
from nidar.target import Target


def test_mvp_end_to_end_flow() -> None:
    """Simulate the complete September 30 MVP: detect → validate → deduplicate → deliver → complete.

    Flow:
        1. Scout drone detects survivors (raw PixelDetection objects).
        2. Wrap each detection in a Target for validation and tracking.
        3. Deduplicator filters invalid targets and merges spatial duplicates.
        4. Create a DeliveryTask for each deduplicated target.
        5. Delivery drone receives and completes each task.
        6. Verify all tasks are in 'completed' status.
    """

    # === STAGE 1: Scout Detection ===
    # Simulate a scout drone flying over the flooded area.
    scout = ScoutDrone(frame_width=1280, frame_height=720)
    raw_detections = scout.detect_survivors()

    # In simulation, scout always returns exactly 1 detection at frame centre.
    assert len(raw_detections) == 1
    assert raw_detections[0].confidence == 0.95

    # === STAGE 2: Target Validation ===
    # Wrap each raw detection in a Target object (assign ID, validate confidence).
    targets = []
    for idx, detection in enumerate(raw_detections):
        target = Target(
            target_id=idx + 1,
            detection=detection,
            confidence_threshold=0.5,
        )
        targets.append(target)

    assert len(targets) == 1
    assert targets[0].is_valid() is True

    # === STAGE 3: Deduplication ===
    # Filter invalid targets and merge spatial duplicates in pixel space.
    deduplicator = TargetDeduplicator(spatial_threshold_pixels=50.0)
    deduplicated_targets = deduplicator.deduplicate(targets)

    # After deduplication, we still have 1 target (no duplicates to merge).
    assert len(deduplicated_targets) == 1
    assert deduplicated_targets[0].target_id == 1
    assert deduplicated_targets[0].detection.confidence == 0.95

    # === STAGE 4: Task Creation ===
    # Create a DeliveryTask for each deduplicated target.
    delivery_tasks = []
    for idx, target in enumerate(deduplicated_targets):
        task = DeliveryTask(task_id=1000 + idx, target=target)
        # Task starts in 'pending' status; transition to 'assigned'.
        assigned_task = task.assign()
        delivery_tasks.append(assigned_task)

    assert len(delivery_tasks) == 1
    assert delivery_tasks[0].status == "assigned"

    # === STAGE 5: Delivery ===
    # Delivery drone receives and completes each task.
    delivery_drone = DeliveryDrone(drone_id=1)
    completed_tasks = []
    for task in delivery_tasks:
        completed_task = delivery_drone.deliver(task)
        completed_tasks.append(completed_task)

    # === VERIFICATION: MVP Complete ===
    # All tasks should be in 'completed' status.
    assert len(completed_tasks) == 1
    assert completed_tasks[0].status == "completed"
    assert completed_tasks[0].target.detection.confidence == 0.95

    # The delivery was successful: one survivor detected, validated, deduplicated,
    # tasked, and delivered.