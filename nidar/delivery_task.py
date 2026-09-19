"""Delivery task management — tracks delivery status for each target."""

from dataclasses import dataclass

from nidar.target import Target


@dataclass(frozen=True)
class DeliveryTask:
    """A delivery task assigned to the delivery drone.

    Attributes:
        task_id: Unique integer identifier for this task.
        target: The Target to be delivered to.
        status: Current status ("pending", "assigned", or "completed").
    """

    task_id: int
    target: Target
    status: str = "pending"

    def __post_init__(self) -> None:
        if self.task_id < 0:
            raise ValueError(f"task_id must be non-negative, got {self.task_id}")

        valid_statuses = {"pending", "assigned", "completed"}
        if self.status not in valid_statuses:
            raise ValueError(
                f"status must be one of {valid_statuses}, got '{self.status}'"
            )

    def assign(self) -> "DeliveryTask":
        """Transition task from 'pending' to 'assigned'.

        Returns:
            A new DeliveryTask with status='assigned'.

        Raises:
            ValueError: If the task is not currently 'pending'.
        """
        if self.status != "pending":
            raise ValueError(
                f"Cannot assign a task with status '{self.status}'; must be 'pending'"
            )
        return DeliveryTask(task_id=self.task_id, target=self.target, status="assigned")

    def complete(self) -> "DeliveryTask":
        """Transition task from 'assigned' to 'completed'.

        Returns:
            A new DeliveryTask with status='completed'.

        Raises:
            ValueError: If the task is not currently 'assigned'.
        """
        if self.status != "assigned":
            raise ValueError(
                f"Cannot complete a task with status '{self.status}'; must be 'assigned'"
            )
        return DeliveryTask(task_id=self.task_id, target=self.target, status="completed")