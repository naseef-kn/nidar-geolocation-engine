"""Deduplication engine — merges spatially close targets in pixel space."""

import math
from dataclasses import dataclass

from nidar.target import Target


@dataclass
class TargetDeduplicator:
    """Removes invalid targets and merges targets that are spatially close.

    Attributes:
        spatial_threshold_pixels: Maximum pixel distance to consider two targets
                                   as the same survivor. Default: 50 pixels.
    """

    spatial_threshold_pixels: float = 50.0

    def __post_init__(self) -> None:
        if self.spatial_threshold_pixels < 0:
            raise ValueError(
                f"spatial_threshold_pixels must be non-negative, got {self.spatial_threshold_pixels}"
            )

    def deduplicate(self, targets: list[Target]) -> list[Target]:
        """Filter invalid targets and merge spatially close ones.

        Args:
            targets: List of Target objects to deduplicate.

        Returns:
            A deduplicated list of valid targets, sorted by confidence (descending).

        Algorithm:
            1. Filter: keep only targets where is_valid() == True
            2. Sort: by detection.confidence (highest first)
            3. Merge: iterate through sorted targets; keep a target only if it is
               not within spatial_threshold_pixels of any already-kept target
        """
        # Step 1: Filter invalid targets
        valid_targets = [t for t in targets if t.is_valid()]

        if not valid_targets:
            return []

        # Step 2: Sort by confidence (descending)
        sorted_targets = sorted(
            valid_targets,
            key=lambda t: t.detection.confidence,
            reverse=True,
        )

        # Step 3: Deduplicate by spatial proximity
        deduplicated: list[Target] = []
        for target in sorted_targets:
            # Check if this target is close to any already-kept target
            is_duplicate = False
            for kept_target in deduplicated:
                distance = self._pixel_distance(
                    target.detection, kept_target.detection
                )
                if distance <= self.spatial_threshold_pixels:
                    is_duplicate = True
                    break

            if not is_duplicate:
                deduplicated.append(target)

        return deduplicated

    @staticmethod
    def _pixel_distance(detection1, detection2) -> float:
        """Calculate Euclidean distance between two detections in pixel space.

        Args:
            detection1: First PixelDetection.
            detection2: Second PixelDetection.

        Returns:
            Euclidean distance: sqrt((x1-x2)^2 + (y1-y2)^2)
        """
        dx = detection1.x - detection2.x
        dy = detection1.y - detection2.y
        return math.sqrt(dx * dx + dy * dy)