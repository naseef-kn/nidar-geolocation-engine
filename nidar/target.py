"""Target representation — a validated survivor detection with tracking metadata."""

from dataclasses import dataclass

from nidar.detection import PixelDetection


@dataclass(frozen=True)
class Target:
    """A validated survivor detection assigned a unique ID for tracking.

    Attributes:
        target_id: Unique integer identifier for this target.
        detection: The underlying PixelDetection object.
        confidence_threshold: Minimum confidence to consider the detection valid (0.0–1.0).
    """

    target_id: int
    detection: PixelDetection
    confidence_threshold: float = 0.5

    def __post_init__(self) -> None:
        if self.target_id < 0:
            raise ValueError(
                f"target_id must be non-negative, got {self.target_id}"
            )
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError(
                f"confidence_threshold must be within 0.0..1.0, got {self.confidence_threshold}"
            )

    def is_valid(self) -> bool:
        """Check if the detection meets the confidence threshold.

        Returns:
            True if detection.confidence >= confidence_threshold, False otherwise.
        """
        return self.detection.confidence >= self.confidence_threshold