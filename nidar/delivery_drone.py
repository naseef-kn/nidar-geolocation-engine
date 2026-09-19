"""Delivery drone simulation — receives tasks and completes deliveries."""

from dataclasses import dataclass

from nidar.delivery_task import DeliveryTask


@dataclass
class DeliveryDrone:
    """Simulated delivery drone that completes delivery tasks.

    Attributes:
        drone_id: Unique identifier for this delivery drone.
    """

    drone_id: int

    def __post_init__(self) -> None:
        if self.drone_id < 0:
            raise ValueError(f"drone_id must be non-negative, got {self.drone_id}")

    def deliver(self, task: DeliveryTask) -> DeliveryTask:
        """Simulate delivering to the target and completing the task.

        Args:
            task: A DeliveryTask in 'assigned' status.

        Returns:
            The same task transitioned to 'completed' status.

        Raises:
            ValueError: If the task is not in 'assigned' status.
        """
        if task.status != "assigned":
            raise ValueError(
                f"Cannot deliver a task with status '{task.status}'; must be 'assigned'"
            )
        return task.complete()